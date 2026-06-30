# GPU Zero-Copy Transport Design

## 背景

* ROS 2 標準のゼロコピー (LoanedMessage 等) は **CPU メモリ**のみを対象とする。
* GPU バッファ (VRAM) をプロセス間・ノード間でコピーせず渡すために、**CUDA IPC** を利用する。
* アプリ側では「画像」「点群」として自然に扱える API を提供しつつ、ROS msg はシンプルで相互運用性が高いものとする。

## 層構造

### 1. 通信メッセージ層 (ROS msg)

#### BufferCore

* メモリ共有方式の識別子（CUDA IPC / VMM + FD）
* CUDA IPC ハンドル (mem_handle, event_handle)
* 参照カウント用共有メモリ名 (shm_name)
* 識別情報 (device_id, slot_id, generation)
* バイトサイズ (byte_size)
* これだけで GPU バッファの受け渡しが可能。

```msg
# ros2_cuda_ipc_msgs/msg/BufferCore.msg

# Memory backend type
uint8 CUDA_IPC=0
uint8 VMM_FD=1
uint8 backend

# CUDA IPC handles / backend payload (opaque bytes)
uint8[64] mem_handle        # cudaIpcMemHandle_t (CUDA_IPC) or uuid bytes (VMM_FD)
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
# ros2_cuda_ipc_msgs/msg/GpuImage.msg
# 画像として扱うために必要な最小限のメタデータを付与。時刻/座標は Header に集約。

std_msgs/Header header         # stamp, frame_id

# 汎用レイアウト（GPU向け、機械可読）
uint8  dtype                    # enum: 0=U8,1=U16,2=F16,3=F32,...
uint32[3] shape                 # {H, W, C}（未使用次元は0でも可）
uint64[3] strides               # バイト単位: {strideH, strideW, strideC}
                                # 典型: strideH = step, strideW = C*sizeof(T), strideC = sizeof(T)

ros2_cuda_ipc_msgs/BufferCore core # data buffer
```
#### GpuPointCloud2
    
* BufferCore を内包。
* 追加情報: width, height, is_dense, point_step, row_step, fields[]。

```msg
# ros2_cuda_ipc_msgs/msg/GpuPointCloud2.msg
# 点群として扱うために必要な最小限のメタデータを付与。時刻/座標は Header に集約。

std_msgs/Header                 header       # stamp, frame_id

# Dimensions & layout
uint32 height
uint32 width

sensor_msgs/PointField[] fields

uint32 point_step                             # bytes per point
uint32 row_step                               # bytes per row (if organized)

# Reuse standard PointField for structure
ros2_cuda_ipc_msgs/BufferCore  core           # data buffer

bool   is_dense
```

memo:
* core.byte_size >= row_step * height
* 各 fields[i].offset + sizeof(type) * count <= point_step
* width * height == N ⇒ point_step > 0

### 2. 内部型 (C++ View)

#### BufferView

  * BufferCore.msg を TypeAdapter が開いた結果 (dev_ptr, ready_evt, LeaseHandle)。
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

  void reset() noexcept;             // dev_ptr/evt をリセットし lease を解放
  void set_ipc_handles(MemoryBackendKind backend,
                       const uint8_t* payload_bytes,
                       std::size_t payload_size,
                       const cudaIpcEventHandle_t& evt) noexcept;
  bool handles_ready() const noexcept;

private:
  MemoryHandlePayload mem_payload_{};  // Publisher から渡されたハンドル
  cudaIpcEventHandle_t event_handle_{};
  bool handles_ready_ = false;
};
```

**方針**

* 同期は提供するが強制しない：enqueue_ready_event(stream) を呼ぶかは利用側の責務。
* ハンドル情報はプロセス内のキャッシュ経由で共有し、BufferView は必要最小限の
  メタデータだけを持つ。
* データの意味付けを持たない：幅やレイアウトは一切保持しない（下位互換性と拡張性を保つため）。

BufferView 自体はハンドルを開かず、受信側 mapper が構築した dev_ptr / ready_evt と
LeaseHandle を保持する。mapper はプロセス内キャッシュを使って同じハンドルの open/import を
初回だけ実行する。TypeAdapter は mapper を呼ぶだけの薄い wrapper とする。これにより
CopyConstructible という rclcpp の要件を満たしつつ、フレームごとの open/close コストを排除する。

#### MemoryBackend の抽象化

MemoryBackend は Publisher 側の slot 確保・破棄を抽象化し、`BufferCore.backend`
で Subscriber 側が使うメモリ共有方式を識別できるようにする。

* `backend=CUDA_IPC` (0): mem\_handle/event\_handle ともに従来どおり CUDA IPC のハンドルを格納する。
* `backend=VMM_FD` (1): mem\_handle には FD を取得するための uuid（最大 64 byte）を格納し、実ハンドルは
  OS レベルで共有した FD から `cuMemImportFromShareableHandle` で復元する。event\_handle は引き続き
  `cudaIpcEventHandle_t` を利用する。

MemoryBackend は「slot 用 GPU メモリの確保」「BufferCore に載せるハンドル情報の生成」
「Publisher 側リソースのクリーンアップ」をまとめる小さなクラスである。Subscriber 側では
`BufferViewMapper` が `BufferCore.backend` を見て CUDA IPC open または VMM import を呼び分ける。
イベントハンドル（`cudaIpcEventHandle_t`）は共通実装を維持し、CUDA IPC ベースの同期を継続する。

##### x86 + dGPU: 既存の cudaIpcMemHandle_t パス

* Publisher は各 Slot の VRAM を `cudaMalloc` で確保し、`cudaIpcGetMemHandle`/`cudaIpcGetEventHandle`
  でハンドルを生成して BufferCore に埋め込む（`backend=CUDA_IPC`）。
* Subscriber は `cudaIpcOpenMemHandle`/`cudaIpcOpenEventHandle` で BufferView を構築し、slot\_id +
  generation で世代管理する実装を据え置く。
* BufferCore.msg の `backend` フィールドが追加された以外は既存の設計と互換であり、x86+dGPU
  環境では MemoryBackend の実装を差し替える必要がない。

##### Jetson Orin: CUDA VMM + FD 配布

Jetson Orin は CUDA IPC のメモリ共有をサポートしていないため、CUDA VMM (`cuMemCreate`) で確保した
GPU メモリを「Shareable FD」として扱い、`backend=VMM_FD` で配布する。

1. **Publisher 側の準備**
   * Slot ごとに uuid（例: `8-4-4-4-12` 形式）を発行し、`mem_handle` に格納する。
   * `cuMemCreate` で確保したハンドルを `cuMemExportToShareableHandle` で FD 化し、uuid ごとに
     `/tmp/cuda_memory_pool_<uuid>.sock` の Unix domain socket を作成。
   * Subscriber からの接続を待ち、`SCM_RIGHTS` で FD を配布する。FD を持つプロセスは
     `cuMemImportFromShareableHandle` で VMM アドレス空間にマップできる。
2. **ROS 2 publish**
   * ROS メッセージには slot\_id、generation、`cudaIpcEventHandle_t` と uuid（mem\_handle）だけを入れる。
     メモリ自体は FD 配布済みなので、フレームごとに再送する必要がない。
   * Subscriber は `mem_handle` から uuid を取り出し、まだ FD を import していなければ Unix ソケット経由で取得
     し、`cuMemImportFromShareableHandle` でデバイスポインタを得る。ready イベントは従来通り
     `cudaIpcOpenEventHandle` で開く。
3. **BufferView と世代管理**
   * slot\_id + generation により遅れて届いた古いメッセージを検出し、その View は無効化する。
     generation mismatch だけを理由に VMM mapping を unmap/reimport しない。
   * VMM mapping を再取得するのは、uuid/allocation が変わった場合、または Publisher 再起動や
     SHM rollover で既存の allocation を使い続けられない場合に限る。
   * Backend が異なる場合でも BufferView の API は同じで、`backend` による切り替えは内部実装に閉じ込める。

4. **VMM + FD 固有のリソース管理**
   * `backend=VMM_FD` では、SHM とは別に Unix domain socket、export 済み FD、import 済み VMM ハンドル、
     VMM アドレス空間の map/unmap を管理する必要がある。
   * Publisher は slot を破棄または再初期化するときに、対応する socket を close/unlink し、export 元の
     CUDA VMM ハンドルを解放する。uuid は slot の allocation を識別する値として扱い、generation の更新だけでは
     socket/FD を作り直さない。
   * Subscriber は import 済み FD と VMM mapping を `IpcHandleCache` で共有する。
     通常の generation 更新ではこのキャッシュを維持し、uuid/allocation の変更や Publisher 再起動、
     SHM rollover を検出した場合にだけ該当 mapping を unmap して再取得する。
   * 現状実装では、UnixFdServer 起動時に bind 前の socket path を `unlink()` し、前回異常終了で残った
     stale socket を取り除く。socket path は uuid により一意である前提だが、所有者確認はまだ行っていない。
   * 今後の課題として、別 Publisher の socket を誤って削除しないよう、shm\_name やプロセス識別子を含む
     管理情報で socket の所有者を確認してから削除する仕組みを検討する。

この設計により、GPU メモリの配布方法だけを backend で差し替え、ROS 2 メッセージ形式と
BufferView/TypeAdapter の API を維持したまま Jetson Orin をサポートできる。明示制御が必要な利用者には
Mapper API を併設する。

#### ImageView（BufferView + 最小限の画像メタデータ）

```cpp
// view/image_view.hpp
#pragma once
#include <cstdint>
#include <string>
#include <ros2_cuda_ipc_core/view/buffer_view.hpp>

namespace ros2_cuda_ipc_core::view {

// 画素型（必要十分な最小セット。用途に応じて拡張可）
enum class DType : uint8_t {
  U8 = 0, U16 = 1,
  F32 = 2, F64 = 3,
  S16 = 4, S32 = 5, U32 = 6
};

struct ImageView {
  // === 開かれたGPUバッファ（refcnt の RAII は LeaseHandle が担う） ===
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
  ~ImageView() = default;                // Lease の解放は core が保持する LeaseHandle に委譲
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

}  // namespace ros2_cuda_ipc_core::view
```

**ポイント**

* POD な DeviceView を用意：そのままカーネル引数に渡せる。
* strideC を持つ：画像処理でピクセル内のチャンネル間隔が重要な場合があるため。
* elem_size_bytes() は簡易版：複雑な型はアプリ側で管理。
* ImageView 自体も BufferView のハンドル情報を共有するためコピー可能。

#### 点群：PointCloud2View（BufferView + レイアウト情報）

```cpp
namespace ros2_cuda_ipc_core::view {

// PointCloud2View: sensor_msgs/PointCloud2 のレイアウトをGPU向けにそのまま保持
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

}  // namespace ros2_cuda_ipc_core::view
```

**ポイント**

* フィールド名はCPU側だけで扱い、GPU には offset/datatype を渡す。
* DeviceField 配列は デバイスメモリに一度コピーしてキャッシュするとよい。
* BufferView がハンドル情報を共有するため、この View もコピー可能。

### 3. 受信変換層 (Mapper + TypeAdapter)

対応表:

* `BufferCore.msg  ⇄ BufferView`
* `GpuImage.msg    ⇄ ImageView`
* `GpuPointCloud2.msg ⇄ PointCloud2View`

Mapper の責務は **Lease の取得、IPC/VMM ハンドルの open/import とキャッシュ、BufferView の構築、
ROS msg と View 間のメタデータコピー**。TypeAdapter の責務は mapper / fill helper を呼ぶことだけに絞る。

* GPU データ本体のコピーは行わない。ROS msg と View の間では、shape や strides などのメタデータだけをコピーする。
* 同期 (cudaStreamWaitEvent) はユーザ側の責務。
* 同一 (mem\_handle,event\_handle) の組み合わせをプロセス内でキャッシュし、
  ハンドルごとの `cudaIpcOpen*Handle` や VMM import を初回だけ実行する。これにより、
  subscriber がフレームごとに IPC open/close を繰り返さずに済み、大きな
  API 開始コストを避けている。
  * キャッシュのライフタイムはプロセス存続中。送信側がメモリを再初期化した
    場合は新しいハンドルが publish されるため、受信側も自然に再オープンする。
  * `BufferView` は破棄時に IPC ハンドルを閉じない。キャッシュと同じ
    ライフタイムで利用する前提のため、ハンドルの解放は送信側に委ねる。

Publisher/Subscriber の役割:
* Publisher: cudaIpcGet*Handle でハンドル生成 → msg に格納。
* Subscriber: Mapper が BufferView を構築し、`BufferCore.backend` に応じて dev_ptr/event を取得する。
* TypeAdapter API を使う場合だけ、TypeAdapter が既定 mapper を呼んで View を返す。

memo:
* cudaIpcOpenEventHandle で開いたイベントの破棄は受信側では不要（破棄は送信側が責任を持つ）。

エラーハンドリングポリシー:
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
    * 一時的なエラーか永続的なエラーかは上位レイヤで判断する。
* Publisher 側での convert_to_ros_message は失敗しない前提（ハンドル生成失敗は publish 前に検出すべき）。

Example:

```cpp
class BufferViewMapper {
 public:
  BufferView map(const ros2_cuda_ipc_msgs::msg::BufferCore& src) const;
};
```

TypeAdapter は mapper に委譲する:

```cpp
static void convert_to_custom(const ros_message_type& src, custom_type& dst) {
  dst = ros2_cuda_ipc_core::map_buffer_view(src);
}
```

### 4. アプリ層

* Publisher
  * バッファプール管理、書き込み、cudaEventRecord、ハンドル取得、ROS msg 生成・publish。
* Subscriber
  * TypeAdapter API では mapper によって初期化済みの View を受け取る。
  * 明示制御 API では raw message を受け取り、`ImageViewMapper::map()` / `PointCloud2ViewMapper::map()` を呼ぶ。
  * アプリは View の `dev_ptr` と `ready_evt` を使い、任意のストリームで同期・処理。

## 命名規則

* **ROS msg**: `BufferCore`, `GpuImage`, `GpuPointCloud2`
* **C++ 内部型**: `BufferView`, `ImageView`, `PointCloud2View`

→ 「View」という語は C++ 側のみで使用。ROS msg 側はシンプルな名前で機能を表す。

## 運用設計のポイント

* **TypeAdapter と Mapper は同期をしない**：Lease 取得と View 初期化のみを行い、ユーザがストリーム同期を行う。
* **Lease RAII 一貫**：View が LeaseHandle を共有し、最後の参照が消えた時点で refcnt を解放する。
  IPC/VMM ハンドルは `IpcHandleCache` のプロセス内キャッシュで共有する。
* **ROS msg の最小メタデータ**：ros2 topic echo / bag で内容をすぐ確認できる程度の情報を追加。
* **TypeNegotiation の余地**：future work。
* エラー発生時は「TypeAdapter エラーハンドリングポリシー」に従う。

## フロー例

1. Publisher:
   * スロットを取得 → GPU に書き込み → cudaEventRecord
   * cudaIpcGet*Handle → BufferCore に格納
   * GpuImage/GpuPointCloud2 を publish
2. Subscriber:
   * ROS msg を受信 → Mapper が View を返す
   * `cudaStreamWaitEvent(my_stream, view.ready_evt, 0)`
   * カーネル呼び出しで view.dev_ptr を利用
