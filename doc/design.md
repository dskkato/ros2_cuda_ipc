#ROS 2 CUDA IPC Zero‑Copy Transport – Design Doc(v1)

**Author:** dskkato
**Date:** 2025‑09‑13
**Status:** Draft (for review)
**Repo (planned):** `dskkato/ros2_cuda_ipc`

---

## 0. Executive Summary

同一ホスト同一GPU上で、ROS 2の別プロセス間におけるGPUメモリのゼロコピー受け渡しを実現する。CUDA IPC（メモリ＋イベント）を用い、DDS上では小さなハンドルとメタデータのみを配送。Publisher がメモリープールを所有・管理し、複数Subscriber（ビデオエンコード／プレビュー生成／DNN推論）が同時に read-only 参照する。

---

## 1. Goals & Non‑Goals

### Goals

* 別プロセス間（ROS 2ノード間）で**ゼロコピー**でGPUメモリを共有
* **同一GPUデバイス前提**（device\_uuid一致チェック）
* Publisher管理の**固定長メモリープール**（スロット数/サイズ上限の設定可能）
* 複数Subscriberの**同時読み取り**（read-only）
* **CUDAイベント**によるproduce完了同期、開放はPublisher主導（Release通知）
* 画像（NV12/BGR/RGBA）・点群（SoA/AoS）をまずサポート
* C++/Python 双方から利用可能（pybind11 + DLPackユーティリティ）

### Non‑Goals

* 異なるGPU/異なるホスト間共有、DMA‑BUF/GPUDirect RDMA 等のサポート
* 書き込み共有（基本はread-only。書込みは将来の派生）
* ROS 2 LoanedMessage/DDS共有メモリとの統合（本設計の対象外）

---

## 2. Requirements

### Functional

* ROS 2メッセージでGPUバッファの**ハンドル＋メタデータ**をpublishする
* Subscriberは初回にIPCハンドルをopen→以後キャッシュ再利用
* Release通知（共有制御ブロック SHM）によりスロット再利用

### Non‑Functional

* **低レイテンシ**：イベント同期 + キャッシュでIPCコスト最小化
* **堅牢性**：Subscriberクラッシュ時のタイムアウト回収
* **観測性**：メトリクス（貸出数、タイムアウト、開放遅延）
* **バージョン互換**：メッセージ/ABIにversion埋め込み

---

## 3. System Architecture

```
[Camera/ColorConvert (Publisher)]
  ├─ GPU Memory Pool (N slots)
  ├─ cudaEvent per slot (produce-ready)
  ├─ Exported cudaIpcMemHandle_t / cudaIpcEventHandle_t
  └─ Publish GpuBuffer msg  ─────►  DDS (ROS 2)  ─────►  [Encoder/Preview/DNN (Subscribers)]
                                                      ├─ Open (cached)
                                                      ├─ cudaStreamWaitEvent
                                                      └─ Use zero-copy & Release
```

### Components

* **ros2\_cuda\_ipc\_msgs**: `GpuBuffer.msg`
* **ros2\_cuda\_ipc\_core**: C++コア（Pool/Mapper、IPCラッパ、管理）
* **ros2\_cuda\_ipc\_py**: pybind11バインディング + DLPackブリッジ
* **sample\_nodes**: Publisher/Subscriber最小実装

---

## 4. Message & Service Definitions

### 4.1 `GpuPlane.msg`

```
uint64 size_bytes
uint64 pitch_bytes
uint8  ipc_mem_handle[64]
```


### 4.2 `GpuBuffer.msg`

```
uint32   abi_version
string   device_uuid  // `cudaDeviceProp::uuid` 文字列
uint64   seq_id       // フレーム番号
uint32   pool_slot_id // プール内スロット
uint8    plane_count
uint8    format       // constants: FORMAT_BGR8, FORMAT_RGBA8, FORMAT_NV12, FORMAT_YUV420, FORMAT_FP16, FORMAT_FP32, FORMAT_PCL_XYZ, ...
uint8    layout       // constants: LAYOUT_LINEAR, LAYOUT_PITCHED, LAYOUT_CHW, LAYOUT_NCHW, LAYOUT_AOS, LAYOUT_SOA
uint32   width
uint32   height
uint32   channels
GpuPlane[] planes     // per plane
uint8   ipc_event_handle[64]
builtin_interfaces/Time stamp
string  frame_id
string  shm_name       // 任意：共有制御ブロック方式を使う場合
```

> 備考: 64バイトはCUDA IPCハンドルの最大サイズ想定。実装時に静的\_assert。

（Release サービスは削除。Release は SHM 制御ブロックの参照カウントで行う）

---

## 5. Publisher‑Side Memory Pool

### 5.1 Options

```c++
struct PoolOptions {
  uint32_t pool_size = 16;                // スロット数
  size_t max_bytes_per_plane = 16 << 20;  // 例: 16 MiB
  uint32_t max_planes = 2;                // NV12想定
  std::chrono::milliseconds lease_timeout{30};
  cudaStream_t producer_stream = 0;  // 省略可
};
```

### 5.2 Slot Structure

```c++
void* dev_ptrs[max_planes]
size_t sizes[max_planes]
size_t pitches[max_planes]
cudaEvent_t ready_evt
cudaIpcMemHandle_t mem_hdl[ax_planes]
cudaIpcEventHandle_t evt_hdl
enum {FREE, IN_USE} + deadline + last_seq_id
```

### 5.3 Lifecycle

1. `borrow(blocking)` で空きスロット取得
2. CPU→GPU転送/色変換/前処理（producer stream）
3. `cudaEventRecord(ready_evt)`
4. `make_message()` で `GpuBuffer` 生成・Publish
5. Release通知 or タイムアウトで `FREE` へ

---

## 6. Subscriber‑Side Mapping

* 初回のみ: `cudaIpcOpenMemHandle` / `cudaIpcOpenEventHandle`（`pool_slot_id`でキャッシュ）
* フレームごと: `cudaStreamWaitEvent(ready_evt)` → 利用 → `release()`（SHM の `refcnt--`）
* キャッシュ失効: Publisherリスタート検知（`abi_version`/`device_uuid`/`seq_id`逆転 など）で `Close + Reopen`

---

## 7. Synchronization & Concurrency

* **produce完了**: `cudaEventRecord` → Subscriberは各自の`cudaStream`で `cudaStreamWaitEvent`
* **読み取り同時実行**: 複数Subscriberが同一スロットを参照可能（read-only）
* **解放**: 共有制御ブロック（SHM）の `refcnt==0` で再利用
* **タイムアウト**: `lease_timeout` 超過で強制再利用（ログ/カウンタ増分）

---

## 8. Data Formats

### 8.1 Images

* Formats: `FORMAT_NV12`, `FORMAT_YUV420`, `FORMAT_BGR8`, `FORMAT_RGBA8`（将来: `FORMAT_P010`, `FORMAT_FP16`）
* Layout: `LAYOUT_LINEAR` or `LAYOUT_PITCHED`（`pitch_bytes` で通知）
* 多平面: 平面ごとに `mem_handle` を渡す。イベントはフレーム単位で1つ。

### 8.2 Point Clouds

* Layout: `AOS`（x,y,z,i, …） / `SOA`（channels × N）
* Precision: `FP32` 推奨（将来: `FP16`）
* メタ: `channels`, `channel_mask`（将来拡張）

---

## 9. ROS 2 Integration

* パッケージ:
  * `ros2_cuda_ipc_msgs`
  * `ros2_cuda_ipc_core`
  * `ros2_cuda_ipc_py`
* QoS: 
  * 映像用途は `best_effort + keep_last(pool_size)` を推奨
* LifecycleNode:
  * `on_configure` でプール確保、`on_cleanup` で破棄
* 名前空間:
  * `/<camera_ns>/gpu_buffers` など

---

## 10. Public APIs (C++)

```c++
// Publisher side
class GpuBufferPool {
 public:
  explicit GpuBufferPool(const PoolOptions &);
  BorrowedSlot borrow();
  GpuBufferMsg make_message(const BorrowedSlot &, const FrameMeta &);
  void on_release(uint64_t seq_id, uint32_t slot_id,
                  std::string_view consumer_id);
};

// Subscriber side
class GpuBufferMapper {
 public:
  OpenedBuffer open(const GpuBufferMsg &msg);  // cached
  void wait_ready(const OpenedBuffer &buf, cudaStream_t user_stream);
  void release(const GpuBufferMsg &msg, std::string_view consumer_id);
};
```

### Python (pybind11)

* `pool = GpuBufferPool(opts)` / `slot = pool.borrow(true)`
* `mapper = GpuBufferMapper()` / `buf = mapper.open(msg)` / `mapper.wait_ready(buf, stream)`
* DLPack: `to_torch(buf, shape, dtype, layout)` / `to_cupy(...)`

---

## 11. Example Sequences

### 11.1 YUV→BGR変換→3 Consumer

```
Publisher (ColorConvert)
  borrow → memcpyAsync(YUV) → kernel_yuv2bgr → EventRecord → Publish(GpuBuffer)

Encoder/Preview/DNN (Subscribers)
  open(cache) → StreamWaitEvent → use_zero_copy → release() [SHM]

Publisher
  refcnt==0（SHM） or timeout → FREE
```

### 11.2 再接続・再初期化

```
Subscriber detects abi_version change or device_uuid mismatch
  → Close all opened handles → Re-open on next message
```

---

## 12. Error Handling & Recovery

* **Device mismatch**: `device_uuid` 不一致→フレームをスキップ、警告
* **cudaIpcOpen\*失敗**: サブスクライバは該当フレームをスキップ
* **Release未達**: タイムアウトで回収（メトリクス記録）
* **プール枯渇**: 設定で `drop_oldest` or `backpressure`
* **Publisher再起動**: `abi_version` でSubscriberに再openを促す

---

## 13. Performance Considerations

* スロット単位の**IPCハンドル再利用**（open/closeコストを回避）
* `PITCHED` を前提に最適化、カーネルは `pitch` 対応
* 可能なら NV12のままエンコーダへ直結（前処理の重複回避）
* メトリクス: フレーム遅延、wait時間、開放までの時間、ドロップ数

---

## 14. Security / Safety

* 同一GPUのみ許可（UUIDチェック）
* 既定は read-only（書込み要求は別型/フラグで明示）
* タイムアウト再利用は**遅延アクセスの危険**があるためログ + アラート

---

## 15. Configuration

```yaml
ros2_cuda_ipc:
  pool_size: 16
  max_bytes_per_plane: 16777216   # 16 MiB
  max_planes: 2
  lease_timeout_ms: 30
  expected_consumers: 3           # 省略可（未指定ならタイムアウト制）
  image:
    default_format: FORMAT_NV12
    layout: LAYOUT_PITCHED
```

---

## 16. Build & Packaging

* **CMake**: `find_package(rclcpp rclpy)`、`CUDA::cuda_driver` 連携
* **Targets**: `libros2_cuda_ipc_core.so`, `ros2_cuda_ipc_msgs`, `ros2_cuda_ipc_py`
* **Python**: `pybind11` + `setuptools`（`scikit-build-core`推奨）

---

## 17. Testing Plan

### Unit

* Pool借用/返却、タイムアウト、メタ生成
* IPC open/closeのキャッシュ動作、異常系（デバイス不一致）

### Integration (on single GPU)

* Publisher + 3 Subscribers のE2E（NV12→BGR→エンコード/縮小/推論ダミー）
* ストレス: 高FPS, 大解像度, 長時間、Subscriberクラッシュ注入

### Performance

* レイテンシ分解: produce→event, event→wait, wait→consume
* プールサイズ/timeout/平面数のスイープ

---

## 18. Rollout Plan

1. **P0**: 画像単平面（BGR/RGBA）、SHM Release、C++のみ
2. **P1**: NV12多平面、pybind11、DLPack
3. **P2**: 点群、監視メトリクス/可視化
4. **P3**: 上位サンプル（FFmpeg NVENC、TensorRT/PyTorch）

---

## 19. Alternatives Considered

* **ROS 2 LoanedMessage / iceoryx**: CPU共有向け。GPUメモリ共有は非対象
* **CUDA MPS**: 同一コンテキスト共有に有用だがプロセス境界でのメモリ共有とは別解
* **DMA‑BUF / EGLStream**: 異プロセス/異デバイスや表示系に強いが依存が重い

---

## 20. Risks & Mitigations

* **Release漏れ**: タイムアウト + SHM refcnt
* **IPCハンドル肥大**: スロット再利用設計でopen頻度を最小化
* **ABIドリフト**: `abi_version` を上げて互換性崩壊を検知

---

## 21. Open Questions

* `expected_consumers` を静的に宣言するか、動的検出するか？
* Python側のストリーム管理（カスタム`cudaStream_t`の公開範囲）
* 点群の標準化（PCL互換メタ or 独自簡素メタ）

---

## 22. Appendix

### 22.1 Minimal Pseudocode – Publisher

```cpp
auto slot = pool.borrow(true);
// memcpyAsync host_yuv → slot.dev_ptrs[0/1]
launch_yuv2bgr_kernel(slot, ...);
cudaEventRecord(slot.ready_evt, producer_stream);
auto msg = pool.make_message(slot, meta);
pub->publish(msg);
```

### 22.2 Minimal Pseudocode – Subscriber

```cpp
auto buf = mapper.open(msg);
mapper.wait_ready(buf, my_stream);
// use buf.dev_ptrs[...] as input (zero-copy)
mapper.release(msg, consumer_id);
```

---

**End of Document**
