# GPU Zero-Copy Transport Design

## 背景

* ROS 2 標準のゼロコピー (LoanedMessage 等) は **CPU メモリ**のみを対象とする。
* GPU バッファ (VRAM) をプロセス間・ノード間でコピーせず渡すために、**CUDA IPC** を利用する。
* アプリ側では「画像」「点群」として自然に扱える API を提供しつつ、ROS msg はシンプルで相互運用性が高いものとする。

## 層構造

### 1. ワイヤ層 (ROS msg)

#### BufferCore

* CUDA IPC ハンドル (mem_handle, event_handle)
* 参照カウント用共有メモリ名 (shm_name)
* 識別情報 (device_id, slot_id, generation)
* バイトサイズ (byte_size)
* これだけで GPU バッファの受け渡しが可能。

```msg
# gpu_zero_copy_msgs/msg/BufferCore.msg

# CUDA IPC handles (opaque bytes)
uint8[64] mem_handle        # cudaIpcMemHandle_t
uint8[64] event_handle      # cudaIpcEventHandle_t

# Shared memory for reference counting
string shm_name

# Identification
uint32 device_id
uint32 slot_id
uint32 generation

# Size
uint64 byte_size
```

#### GpuImage

* BufferCore を内包。
* 追加情報: dtype, shape, strides（GPU向け汎用レイアウト）。

```msg
# gpu_zero_copy_msgs/msg/GpuImage.msg
# 画像としての薄メタを付与。時刻/座標は Header に集約。

std_msgs/Header header         # stamp, frame_id

# 汎用レイアウト（GPU向け、機械可読）
uint8  dtype                    # enum: 0=U8,1=U16,2=F16,3=F32,...
uint32[3] shape                 # {H, W, C}（未使用次元は0でも可）
uint64[3] strides               # バイト単位: {strideH, strideW, strideC}
                                # 典型: strideH = step, strideW = C*sizeof(T), strideC = sizeof(T)

gpu_zero_copy_msgs/BufferCore core # data buffer
```
#### GpuPointCloud2
    
* BufferCore を内包。
* 追加情報: width, height, is_dense, point_step, row_step, fields[]。

```msg
# gpu_zero_copy_msgs/msg/GpuPointCloud2.msg
# 点群としての薄メタを付与。時刻/座標は Header に集約。

std_msgs/Header                 header       # stamp, frame_id

# Dimensions & layout
uint32 height
uint32 width

sensor_msgs/PointField[] fields

uint32 point_step                             # bytes per point
uint32 row_step                               # bytes per row (if organized)

# Reuse standard PointField for structure
gpu_zero_copy_msgs/BufferCore  core           # data buffer

bool   is_dense
```

memo:
* core.byte_size >= row_step * height
* 各 fields[i].offset + sizeof(type) * count <= point_step
* width * height == N ⇒ point_step > 0

### 2. 内部型 (C++ View)

#### BufferView

  * BufferCore.msg を開いた状態 (dev_ptr, ready_evt, RAII)。
  * データの解釈は持たない。

```cpp
// BufferView: GPUバッファそのものの借用ビュー（意味付けなし）
struct BufferView {
  // === リソース（必ず有効時は同一 device 上） ===
  void*       dev_ptr = nullptr;     // デバイス先頭
  cudaEvent_t ready_evt = nullptr;   // "書き終わり" を示すイベント（他プロセス発行）
  int         device_id = 0;
  uint64_t    byte_size = 0;

  // === 運用メタ（安全・再利用のため） ===
  uint32_t slot_id = 0;
  uint32_t generation = 0;
  std::string shm_name;                // LeaseHandle のキー
  std::shared_ptr<LeaseHandle> lease;  // 共有メモリrefcntのRAIIハンドル

  // === ライフサイクル ===
  BufferView() = default;
  ~BufferView();
  BufferView(const BufferView&);
  BufferView& operator=(const BufferView&);
  BufferView(BufferView&&) noexcept;
  BufferView& operator=(BufferView&&) noexcept;

  // === 最小アクセサ／ユーティリティ ===
  template<class T = void> T* data() const noexcept { return static_cast<T*>(dev_ptr); }
  bool valid() const noexcept { return dev_ptr != nullptr; }

  // 自分のストリームに書き終わりイベントを依存として積む
  cudaError_t enqueue_ready_event(cudaStream_t s) const noexcept {
    return ready_evt ? cudaStreamWaitEvent(s, ready_evt, 0) : cudaSuccess;
  }

  void reset() noexcept;             // dev_ptr/evt を破棄し lease を解放
  void set_ipc_handles(const cudaIpcMemHandle_t& mem,
                       const cudaIpcEventHandle_t& evt) noexcept;
  bool handles_ready() const noexcept;

private:
  struct ControlBlock {
    void* dev_ptr = nullptr;
    cudaEvent_t ready_evt = nullptr;
    bool opened_mem_via_ipc = false;
    bool opened_event_via_ipc = false;
    ~ControlBlock();                 // cudaIpcCloseMemHandle / cudaEventDestroy
  };

  void ensure_control_block() noexcept;  // コピー時に共有する shared_ptr を確保
  std::shared_ptr<ControlBlock> control_;
  cudaIpcMemHandle_t mem_handle_{};      // Publisher から渡されたハンドル
  cudaIpcEventHandle_t event_handle_{};
  bool handles_ready_ = false;
};
```

**方針**

* 同期は提供するが強制しない：enqueue_ready_event(stream) を呼ぶかは利用側の責務。
* Control block を shared_ptr で共有し、`cudaIpcClose*` を一度だけ実行しつつコピー可能にする。
* 解釈ゼロ：幅やレイアウトは一切持たない（下位互換と拡張性の源）。

Control block は BufferView が保持する開いた CUDA リソース（`dev_ptr`/`ready_evt`）と
「自分で Open したのか？」というフラグを持つ。BufferView をコピーしても Control block
が共有されるため、最後の 1 つが破棄されるタイミングでのみ `cudaIpcCloseMemHandle`
や `cudaEventDestroy` を呼べる。TypeAdapter のユーザー定義型が CopyConstructible である
という rclcpp の要件を満たすための仕組みである。

#### ImageView（BufferView + 薄い解釈）

```cpp
// image_view.hpp
#pragma once
#include <cstdint>
#include <string>
#include "buffer_view.hpp"

// 画素型（必要十分な最小セット。用途に応じて拡張可）
enum class DType : uint8_t {
  U8 = 0, U16 = 1,
  F32 = 2, F64 = 3,
  S16 = 4, S32 = 5, U32 = 6
};

struct ImageView {
  // === 開かれたGPUバッファ（RAIIは BufferView が担保） ===
  BufferView core;

  // === 汎用レイアウト（GPU/機械向け） ===
  // shape = {H, W, C}, strides は "バイト" 単位のストライド
  DType   dtype         = DType::U8;
  uint32_t shape[3]     = {0, 0, 0};     // {rows, cols, channels}
  uint64_t strides[3]   = {0, 0, 0};     // {strideH, strideW, strideC}

  // === 任意：互換/可読（空文字なら未設定） ===
  std::string encoding;                  // 例: "rgb8","mono16","32FC1" 等

  // === BufferView を共有する借用ビュー（コピー可能） ===
  ImageView() = default;
  ~ImageView() = default;                // リソース解放は core の RAII に委譲
  ImageView(const ImageView&) = default;
  ImageView& operator=(const ImageView&) = default;
  ImageView(ImageView&&) noexcept = default;
  ImageView& operator=(ImageView&&) noexcept = default;

  // === 最小アクセサ ===
  uint32_t rows()      const noexcept { return shape[0]; }
  uint32_t cols()      const noexcept { return shape[1]; }
  uint32_t channels()  const noexcept { return shape[2]; }
  uint64_t strideH()   const noexcept { return strides[0]; }  // bytes per row (pitch/step)
  uint64_t strideW()   const noexcept { return strides[1]; }  // bytes per column step
  uint64_t strideC()   const noexcept { return strides[2]; }  // bytes per channel step
  bool     valid()     const noexcept { return core.valid() && rows()>0 && cols()>0; }

  // 画素1要素あたりのバイト数（簡易版）
  uint32_t elem_size_bytes() const noexcept {
    switch (dtype) {
      case DType::U8:  return 1;
      case DType::U16: return 2;
      case DType::F32: return 4;
      case DType::F64: return 8;
      case DType::S16: return 2;
      case DType::S32: return 4;
      case DType::U32: return 4;
    }
    return 1;
  }

  // 同期依存の登録（必要なら呼ぶ。どのストリームで待つかは呼び手が決める）
  cudaError_t enqueue_ready_event(cudaStream_t s) const noexcept {
    return core.enqueue_ready_event(s);
  }

  // === カーネルに渡すPODビュー（ABIを安定させるなら別ヘッダで固定化推奨） ===
  struct DeviceView {
    uint8_t* data;            // base ptr
    int      height, width, channels;
    uint64_t strideH, strideW, strideC; // bytes
    uint8_t  dtype;           // DType の値（uint8_tに格納）
  };

  DeviceView as_device_view() const noexcept {
    return DeviceView{
      core.data<uint8_t>(),
      static_cast<int>(rows()),
      static_cast<int>(cols()),
      static_cast<int>(channels()),
      strideH(), strideW(), strideC(),
      static_cast<uint8_t>(dtype)
    };
  }

  // === 防御的チェック（任意で利用） ===
  bool sanity_check() const noexcept {
    if (!valid() || rows()==0 || cols()==0 || channels()==0) return false;
    const uint64_t last_row  = (rows()-1)     * strideH();
    const uint64_t last_col  = (cols()-1)     * strideW();
    const uint64_t last_chan = (channels()-1) * strideC();
    const uint64_t needed    = last_row + last_col + last_chan + elem_size_bytes();
    return core.byte_size >= needed;
  }
};
```

**ポイント**

* POD な DeviceView を用意：そのまま kernel 引数に渡せる。
* strideC を持つ：画像処理でピクセル内のチャンネル間隔が重要な場合があるため。
* elem_size_bytes() は簡易版：複雑な型はアプリ側で管理。
* ImageView 自体も BufferView の Control block を共有するためコピー可能。

#### 点群：PointCloud2View（BufferView + layout）

```cpp
// PointCloud2View: sensor_msgs/PointCloud2 の layout をGPU向けにそのまま保持
struct PointCloud2View {
  BufferView core;

  // layout（sensor_msgs/PointCloud2 準拠）
  uint32_t height = 1;
  uint32_t width  = 0;
  uint32_t point_step = 0;   // bytes per point
  uint32_t row_step   = 0;   // bytes per row
  bool     is_dense   = true;

  // fields はCPU側メタ。GPUカーネルでは name 参照は避け、offset/datatype に落とす
  struct Field {
    std::string name;
    uint32_t offset;
    uint8_t datatype;
    uint32_t count;
  };
  std::vector<Field> fields;

  // よく使うフィールドのオフセットをキャッシュしておくと便利（任意）
  int x_off=-1, y_off=-1, z_off=-1, intensity_off=-1, rgb_off=-1;

  // デバイス側POD（文字列を排除したメタ）
  struct DeviceField {
    uint32_t offset;
    uint8_t datatype;
    uint32_t count;
  };
  struct DeviceView {
    uint8_t* data;
    int      width, height;
    size_t   point_step, row_step;
    bool     is_dense;
    // 可変長fieldsは用途に応じて最大数を決める or 別バッファで渡す
    const DeviceField* fields; // GPU側にコピー済みのメタ配列を指す想定
    int      num_fields;
  };

  // デバイス用メタ配列の用意は呼び出し側の責務（頻繁に変わらないのでキャッシュ推奨）
  DeviceView as_device_view(const DeviceField* d_fields, int n) const noexcept {
    return DeviceView{
      core.data<uint8_t>(),
      static_cast<int>(width),
      static_cast<int>(height),
      point_step, row_step,
      is_dense,
      d_fields, n
    };
  }

  size_t num_points() const noexcept { return static_cast<size_t>(width) * height; }
  cudaError_t enqueue_ready_event(cudaStream_t s) const noexcept {
    return core.enqueue_ready_event(s);
  }
};
```

**ポイント**

* フィールド名はCPU側だけで扱い、GPU には offset/datatype を渡す。
* DeviceField 配列は デバイスメモリに一度コピーしてキャッシュするとよい。
* BufferView が Control block を共有するため、この View もコピー可能。

### 3. 変換層 (TypeAdapter)

対応表:

* `BufferCore.msg  ⇄ BufferView`
* `GpuImage.msg    ⇄ ImageView`
* `GpuPointCloud2.msg ⇄ PointCloud2View`

Adapter の責務は **Open/Close とメタ転写のみ**。

* コピーは行わない。
* 同期 (cudaStreamWaitEvent) はユーザ側の責務。

publisher/subscriberの役割:
* Publisher: cudaIpcGet*Handle でハンドル生成 → msg に格納。
* Subscriber: cudaIpcOpen*Handle で dev_ptr/event を開く。

memo:
* cudaIpcOpenEventHandle で開いた イベントの破棄は受信側では不要（破棄は送信側が責任）。

Error handling ポリシー:
convert_to_custom (ROS→View) で cudaIpcOpen*Handle する際の失敗ケースと方針について:

**主な失敗ケース**  
1. ハンドル期限切れ / 送信側で解放済み
    * cudaErrorInvalidResourceHandle が返る。
2. デバイス不一致（別 GPU に open しようとした）
    * cudaErrorInvalidDevice。
3. 世代不一致 / バッファ再利用による衝突
    * 自前で slot_id + generation を比較し検出する。
4. IPC Event が開けない
    * 同様に cudaErrorInvalidResourceHandle。

**推奨ポリシー**
* convert_to_custom 内では例外を投げない。
  * rclcpp::TypeAdapter 経由だと例外伝播が難しい。
  * 代わりに「無効な View (dev_ptr==nullptr)」を返す。
* Subscriber 側のコールバックで view.valid() を必ず確認。
  * 無効ならログ出力して破棄。
  * QoS が reliable でも再送は要求しない（上位レイヤの責務にする）。
    * Is this a transient error? or permanent?
* Publisher 側での convert_to_ros_message は失敗しない前提（ハンドル生成失敗は publish 前に検出すべき）。

Example:

```cpp
// type_adapter_buffer_view.hpp
#pragma once
#include <rclcpp/type_adapter.hpp>
#include <gpu_zero_copy_msgs/msg/buffer_core.hpp>
#include "buffer_view.hpp"

namespace rclcpp {
template<>
struct TypeAdapter<BufferView, gpu_zero_copy_msgs::msg::BufferCore> {
  using is_specialized = std::true_type;
  using custom_type = BufferView;
  using ros_message_type = gpu_zero_copy_msgs::msg::BufferCore;

  // Subscriber側：ROS→View（ここで Open＋refcnt++）
  static void convert_to_custom(const ros_message_type& src, custom_type& dst);

  // Publisher側：View→ROS（ハンドル生成は別レイヤで、ここはメタ転写のみ）
  static void convert_to_ros_message(const custom_type& src, ros_message_type& dst);
};
} // namespace rclcpp
```

Implementation (convert_to_custom):

```cpp
static void convert_to_custom(const ros_message_type& src, custom_type& dst) {
  // try open memory
  void* ptr = nullptr;
  auto err = cudaIpcOpenMemHandle(&ptr, src.mem_handle, cudaIpcMemLazyEnablePeerAccess);
  if (err != cudaSuccess) {
    RCLCPP_WARN(rclcpp::get_logger("BufferView"),
                "Failed to open CUDA IPC handle: %s", cudaGetErrorString(err));
    dst = BufferView{};  // invalid view
    return;
  }

  // 同様に event handle を開く
  cudaEvent_t evt{};
  err = cudaIpcOpenEventHandle(&evt, src.event_handle);
  if (err != cudaSuccess) {
    RCLCPP_WARN(rclcpp::get_logger("BufferView"),
                "Failed to open CUDA IPC event handle: %s", cudaGetErrorString(err));
    cudaIpcCloseMemHandle(ptr);
    dst = BufferView{};
    return;
  }

  // 正常なら view を構築
  dst.dev_ptr = ptr;
  dst.ready_evt = evt;
  dst.device_id = src.device_id;
  dst.byte_size = src.byte_size;
  dst.slot_id = src.slot_id;
  dst.generation = src.generation;
  // lease 管理は別処理で attach
}
```

TODO:
* Adapter のテンプレート化（ImageView, PointCloud2View）

### 4. アプリ層

* Publisher
  * バッファプール管理, 書き込み, cudaEventRecord, ハンドル取得, ROS msg 生成・publish。
* Subscriber
  * TypeAdapter によって Open 済みの View を受け取る。
  * アプリは View の `dev_ptr` と `ready_evt` を使い、任意のストリームで同期・処理。

## 命名規則

* **ROS msg**: `BufferCore`, `GpuImage`, `GpuPointCloud2`
* **C++ 内部型**: `BufferView`, `ImageView`, `PointCloud2View`

→ 「View」という語は C++ 側のみで使用。ROS msg 側はシンプルな名前で機能を表す。

## 運用設計のポイント

* **Adapter は同期をしない**：Open/Close のみ。ユーザがストリーム同期を行う。
* **RAII 一貫**：View の寿命 = ハンドル寿命。Control block と LeaseHandle を共有しつつコピー可。
* **ROS msg の薄メタ**：ros2 topic echo / bag で内容が即読める程度に情報を追加。
* **TypeNegotiation 余地**： future work。
* エラー発生時は「TypeAdapterエラーハンドリングポリシー」に従う

## フロー例

1. Publisher:
   * スロットを取得 → GPU に書き込み → cudaEventRecord
   * cudaIpcGet*Handle → BufferCore に格納
   * GpuImage/GpuPointCloud2 を publish
2. Subscriber:
   * ROS msg を受信 → Adapter が View を返す
   * `cudaStreamWaitEvent(my_stream, view.ready_evt, 0)`
   * カーネル呼び出しで view.dev_ptr を利用
