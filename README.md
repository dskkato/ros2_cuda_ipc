# ros2_cuda_ipc

ROS 2 CUDA IPC Zero-Copy Transport

## 概要

`ros2_cuda_ipc` は、**ROS 2 ノード間で GPU メモリをゼロコピー共有**するためのライブラリ／メッセージ定義を提供します。
同一ホスト・同一GPUデバイスを前提に、CUDA IPC（`cudaIpcMemHandle_t` / `cudaIpcEventHandle_t`）を利用し、Publisher ノードが所有するメモリープールを Subscriber が read-only 参照できます。

主な用途は以下です：
- カメラドライバー → CUDA 前処理 → 複数ノード（エンコード／プレビュー／DNN推論）へのゼロコピー分配
- 画像（NV12, BGR, RGBA 等）、点群（AoS/SoA）など大規模GPUデータの効率的な伝搬
- C++ / Python 双方から利用可能

詳細な設計は [doc/design.md](doc/design.md) を参照してください。

## パッケージ構成

このリポジトリは以下の ROS 2 パッケージを含みます：

- `ros2_cuda_ipc_msgs` — GPU バッファ共有のためのメッセージ定義
- `ros2_cuda_ipc_core` — メモリープール、CUDA IPCユーティリティ、マッパの C++ 実装（CUDA 前提）
- `sample_nodes` — メッセージ送受信を確認する簡単なサンプルノード

## 開発環境セットアップ

### 前提条件
- Ubuntu 22.04 (推奨)
- ROS 2 Humble 以降
- CUDA Toolkit 11.8+（ドライバは対応するもの）
- CMake 3.16+
- Python 3.10+
- `colcon` ビルドツール
- 開発用依存:
  - `pybind11`
  - `ament_cmake`, `ament_cmake_python`
  - `rclcpp`, `rclpy`
  - `sensor_msgs` (依存例)

### ビルド手順

```bash
# リポジトリを取得
git clone https://github.com/t2-sw/ros2_cuda_ipc.git
cd ros2_cuda_ipc

# ROS 2 環境をsource
source /opt/ros/humble/setup.bash

# ビルド
colcon build --symlink-install

# ビルド成果を反映
source install/setup.bash
```

### サンプルノード実行（単平面・SHMリリース・イベント同期）

```bash
# Publisher
ros2 run sample_nodes gpu_buffer_publisher

# Subscriber
ros2 run sample_nodes gpu_buffer_subscriber
```

期待される動作（CUDA 環境）
- Publisher: スロットを借用し、GPUメモリの IPC ハンドルとイベントハンドルを `GpuBuffer` に格納して publish。スロットは SHM 制御ブロックの参照カウントが 0 になると解放（`lease_timeout_ms` 過ぎると強制解放、既定 3000ms）。
- Subscriber: 受信したハンドルをキャッシュ（`GpuBufferMapper`）し、自身の CUDA ストリームで `cudaStreamWaitEvent`。処理後に SHM の参照カウントを原子的にデクリメント。
- ログ: Subscriber 側で `Event waited ~X ms` が出力。Publisher 側で `Publishing seq=... (slot X)` と `Slot X freed ... via SHM` が出力。

パラメータ
- Publisher: `lease_timeout_ms`（既定 3000）。例: `ros2 run sample_nodes gpu_buffer_publisher --ros-args -p lease_timeout_ms:=1000`
- Publisher: `shm_owner`（省略時は `sanitized(FQN)_<epoch>_<pid>` を自動生成）。複数Publisher共存時の SHM 名衝突回避に使用。

備考
- Publisher はメッセージに `abi_version` と `device_uuid` を埋め込みます。Subscriber はそれらが変化した場合にマッピングキャッシュを自動リセットします。

## ゼロコピーラッパ API（推奨）

Publisher / Subscriber 双方で、最小コードでゼロコピー通信を行えるラッパを提供します。

- Publisher 側: `ros2_cuda_ipc_core::ZeroCopyPublisher`
- Subscriber 側: `ros2_cuda_ipc_core::ZeroCopySubscriber`

### Publisher: ZeroCopyPublisher の使い方

初期化（プールやイベント、プロデューサストリームの設定）:

```cpp
ros2_cuda_ipc_core::PoolOptions opts;
opts.pool_size = 2;                 // スロット数
opts.bytes_per_slot = 4*1024*1024;  // スロットサイズ（GPUメモリ確保）
opts.events_enabled = true;         // イベント同期
opts.producer_stream = ros2_cuda_ipc_core::cuda_is_available()
                           ? ros2_cuda_ipc_core::cuda_stream_create()
                           : nullptr;

ros2_cuda_ipc_core::ZeroCopyPublisher zcp(opts, /*lease_timeout_ms=*/3000, /*shm_owner=*/"my_pub");
```

1 発行の流れ（借用→GPU 書込→イベント記録→SHM リース→publish）:

```cpp
ros2_cuda_ipc_msgs::msg::GpuBuffer msg;
msg.seq_id = seq++;
msg.layout = ros2_cuda_ipc_msgs::msg::GpuBuffer::LAYOUT_LINEAR;
msg.format = ros2_cuda_ipc_msgs::msg::GpuBuffer::FORMAT_BGR8;
msg.width = 640; msg.height = 480; msg.channels = 3;
msg.stamp = now(); msg.frame_id = "frame";

int expected = static_cast<int>(pub->get_subscription_count()); // -1 で自動指定可

auto fn = [](void* dev, uint32_t w, uint32_t h, uint32_t c, cudaStream_t s){
  const uint64_t bytes = static_cast<uint64_t>(w)*h*c;
  // ここで GPU へ書き込み（例: cudaMemsetAsync 等）
};

bool ok = zcp.produce_and_publish(*pub, msg, expected, fn, /*blocking=*/true);
// ok=false の場合はメタデータのみ publish（plane_count=0）
```

主な引数・挙動:
- `expected_consumers` > 0: SHM リースを開始（各 Subscriber は処理後に参照カウントを decrement）。
- `expected_consumers` == 0: publish 後すぐにスロットを pool に release。
- `expected_consumers` < 0: 呼び出し側で自動的に購読数から決定するなどの前処理を推奨。
- `blocking` が false かつ空きスロットなし: メタデータのみ publish。

### Subscriber: ZeroCopySubscriber の使い方

受信した `GpuBuffer` を 1 呼び出しで open + wait + 処理 + SHM decrement（RAII）:

```cpp
ros2_cuda_ipc_core::ZeroCopySubscriber zcs; // CUDA 環境なら内部で stream を生成

sub = this->create_subscription<GpuBuffer>("gpu_buffer", 10,
  [&](const GpuBuffer& msg){
    zcs.consume(msg, [](void* dev, uint32_t w, uint32_t h, uint32_t c, cudaStream_t s){
      // dev を読み取り専用で使用。処理を s 上に enqueue。
    }, /*sync_on_dtor=*/true);
  });
```

内部では `GpuBufferMapper` のキャッシュを用いて IPC ハンドルの open を再利用し、イベントは `cudaStreamWaitEvent` で同期します。破棄時に SHM 参照カウントを 1 つ減らします。

---

## 開発フロー

* 設計は [doc/design.md](doc/design.md) に従う
* Issue / PR ベースで進める
* コードスタイル: clang-format, ament_lint
* CI: GitHub Actions (予定)

---

## ライセンス

Apache License 2.0

---

## P0 ステータス（実装済み範囲）
- 単平面ゼロコピー（mem IPC ハンドル + イベント IPC ハンドルの配送）
- イベント同期（Publisher: producer stream で記録、Subscriber: `cudaStreamWaitEvent`）
- SHM 制御ブロック（参照カウント）によるスロット解放 + `lease_timeout_ms` による強制回収
- `GpuBufferMapper` による mem/event の open キャッシュ
- `abi_version` / `device_uuid` 変化時の Subscriber 側キャッシュ失効

制約（P0）
- 同一ホスト・同一 GPU のみ（UUID/PCI情報で検査）
- C++ のみ（Python/DLPack は今後）
- 多平面（NV12等）は今後拡張

## CUDA について（coreは CUDA 前提）

- `ros2_cuda_ipc_core` は CUDA を前提にビルドされます（`CUDAToolkit` が必須）。
- サンプルの擬似GPUワーク（待機時間可視化）は任意です。無効化したい場合:

```bash
colcon build \
  --packages-select sample_nodes \
  --cmake-args -DSAMPLE_NODES_ENABLE_CUDA=OFF
```

- テスト: `ros2_cuda_ipc_core` には以下の gtest が含まれます（環境により skip あり）
  - `test_cuda_support`（CUDA ありで実行）
  - `test_gpu_buffer_pool`（CUDA 依存なし）
  - `test_gpu_buffer_mapper`（一部環境で同一プロセスの IPC event open が不可 → スキップ分岐あり）
  - `test_lease_manager`（SHM リリースとタイムアウト解放を確認）
  - `test_scoped_mapped_frame`（破棄時の SHM decrement を確認）

- サンプル側のテスト: なし（サンプルは実行用）。

ビルドとテスト実行例:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select ros2_cuda_ipc_core sample_nodes --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select ros2_cuda_ipc_core sample_nodes
colcon test-result --verbose
```
