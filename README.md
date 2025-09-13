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

- `ros2_cuda_ipc_msgs` — GPU バッファ共有のためのメッセージ／サービス定義
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

### サンプルノード実行（単平面・Releaseサービス・イベント同期）

```bash
# Publisher
ros2 run sample_nodes gpu_buffer_publisher

# Subscriber
ros2 run sample_nodes gpu_buffer_subscriber
```

期待される動作（CUDA 環境）
- Publisher: スロットを借用し、GPUメモリの IPC ハンドルとイベントハンドルを `GpuBuffer` に格納して publish。スロットは Release サービスで解放されるまで保持（`lease_timeout_ms` 過ぎると強制解放、既定 3000ms）。
- Subscriber: 受信したハンドルをキャッシュ（`GpuBufferMapper`）し、自身の CUDA ストリームで `cudaStreamWaitEvent`。処理後に Release サービスを呼び出しスロット解放。
- ログ: Subscriber 側で `Event waited ~X ms` が出力。Publisher 側で `Publishing seq=... (slot X)` と Release 受信ログが出力。

パラメータ
- Publisher: `lease_timeout_ms`（既定 3000）。例: `ros2 run sample_nodes gpu_buffer_publisher --ros-args -p lease_timeout_ms:=1000`

備考
- Publisher はメッセージに `abi_version` と `device_uuid` を埋め込みます。Subscriber はそれらが変化した場合にマッピングキャッシュを自動リセットします。

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
- Release サービスによるスロット解放 + `lease_timeout_ms` による強制回収
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

- テスト: `test_cuda_support` / `test_gpu_buffer_pool` / `test_gpu_buffer_mapper` を含みます（環境により skip あり）。
