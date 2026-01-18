# ros2_cuda_ipc

ROS 2 CUDA IPC Zero-Copy Transport

## Julia Set デモ手順

```bash
ros2 launch julia_set julia_set_demo.launch.py
```

`rqt` を起動し、`Plugins` → `Visualization` → `Image View` を開いたあと、`Topic` を再読み込みして `/julia_set/image_cpu` を選択すると CPU イメージを確認できます。

![julia_set_demo](doc/media/julia_readme.gif)

<details> <summary>動画作成方法の説明</summary>

▲ `rosbag2_2025_09_28-23_34_20` から `/julia_set/image_cpu` を抜粋し、約 3 秒に間引いてエンコードしています。

再生成するときは、`doc/scripts/generate_bag_preview.py` を実行します。

```bash
python3 doc/scripts/generate_bag_preview.py \
  rosbag2_YYYY_MM_DD-hh_mm_ss --output doc/media/julia_readme.mp4 \
  --crf 28 --fps 10 --frame-stride 2
```

</details>

## 概要

`ros2_cuda_ipc` は、**ROS 2 ノード間で GPU メモリをゼロコピー共有**するためのライブラリ／メッセージ定義を提供します。
同一ホスト・同一GPUデバイスを前提に、CUDA IPC（`cudaIpcMemHandle_t` / `cudaIpcEventHandle_t`）を利用し、Publisher ノードが所有するメモリープールを Subscriber が read-only 参照できます。

主な用途は以下です：
- カメラドライバー → CUDA 前処理 → 複数ノード（エンコード／プレビュー／DNN推論）へのゼロコピー分配
- 画像（NV12, BGR, RGBA 等）、点群（AoS/SoA）など大規模GPUデータの効率的な伝搬
- C++ アプリケーションからの GPU メモリ共有

詳細な設計は [doc/design.md](doc/design.md) を参照してください。

## パッケージ構成

このリポジトリは以下の ROS 2 パッケージを含みます：

- `ros2_cuda_ipc_msgs` — GPU バッファ共有のためのメッセージ定義
- `ros2_cuda_ipc_core` — メモリープール、CUDA IPCユーティリティ、マッパの C++ 実装（CUDA 前提）
- `sample_nodes` — メッセージ送受信を確認する簡単なサンプルノード
- `ros2_cuda_ipc_test` — CUDA IPC と VMM-FD の動作テストアプリケーション（ROS2非依存）
- `gpu_image_transport` — GPU画像転送用のimage_transport プラグイン
- `julia_set` — Julia集合のGPU描画デモノード

## 開発環境セットアップ

### 前提条件
- Ubuntu 22.04 (推奨)
- ROS 2 Humble 以降
- CUDA Toolkit 11.8+（ドライバは対応するもの）
- CMake 3.16+
- Python 3.10+
- `colcon` ビルドツール
- 開発用依存:
  - `ament_cmake`, `ament_cmake_python`
  - `rclcpp`
  - `sensor_msgs` (依存例)

### ビルド手順

```bash
# リポジトリを取得
git clone https://github.com/t2-sw/ros2_cuda_ipc.git
cd ros2_cuda_ipc

# ROS 2 環境をsource
source /opt/ros/humble/setup.bash

# CUDA コンパイラのパスを設定（必要な場合）
export CUDACXX=/usr/local/cuda/bin/nvcc

# ビルド
colcon build --symlink-install

# ビルド成果を反映
source install/setup.bash
```

### CUDA IPC 動作テスト

本ライブラリを使用する前に、お使いの環境でCUDA IPCが正しく動作するかテストできます。

```bash
# CUDA IPC テスト（従来のcudaIpcGetMemHandle方式）
ros2 launch ros2_cuda_ipc_test cuda_ipc.launch.py

# VMM-FD テスト（Driver API Virtual Memory Management方式）
ros2 launch ros2_cuda_ipc_test vmm.launch.py
```

詳細は [ros2_cuda_ipc_test/README.md](ros2_cuda_ipc_test/README.md) を参照してください。

### サンプルノード実行

GPU イメージ／ポイントクラウドの 2 系統のサンプルノードを用意しています。どちらも CUDA IPC を使って、Publisher が GPU メモリスロットを確保し、Subscriber がゼロコピー参照します。

```bash
# イメージデモ（Publisher + Subscriber 2 ノード）
ros2 launch sample_nodes gpu_image_demo.launch.py

# ポイントクラウドデモ（Publisher + Subscriber 2 ノード）
ros2 launch sample_nodes gpu_pointcloud_demo.launch.py
```

デモ Launch ファイルでは代表的なパラメータを指定しています。Publisher 単体で起動する場合は `ros2 run` と `--ros-args` で上書きできます。

主なパラメータ（GpuImagePublisher/GpuPointCloudPublisher 共通）
- `publish_rate_hz`: Publish 周期（既定 イメージ 30 Hz / ポイントクラウド 10 Hz）
- `slot_count`: 確保する GPU メモリスロット数（既定 4）
- `pending_ttl_ms`: 未消費スロットを強制解放する猶予時間 [ms]。高フレームレートのカメラでは 80~120ms 程度に設定すると滞留を抑制できます。
- `shm_name`: 共有メモリ領域の名前（用途別にデフォルトを用意）
- `device_index`: 利用する CUDA デバイス（既定 0）

イメージ系の追加パラメータ
- `width`, `height`, `channels`
- `dtype`（`u8`,`u16`,`f32` など）
- `encoding`（ROS 2 画像エンコーディング文字列）

ポイントクラウド系の追加パラメータ
- `width`, `height`
- `is_dense`
- `fill_value`（デモ用に擬似生成する値のベース）

サンプル Subscriber は受信したハンドルを `ros2_cuda_ipc_core` のビュークラス（`ImageView` / `PointCloud2View`）経由でマップし、GPU メモリを直接参照します。

実装上は `sample_nodes/src/gpu_image_publisher_helper.cpp` と `sample_nodes/src/gpu_pointcloud_publisher_helper.cpp` に共通化された処理があり、`pending_ttl` は `std::chrono::milliseconds` で扱われます。

個別起動例:

```bash
ros2 run sample_nodes gpu_image_publisher \
  --ros-args -p pending_ttl_ms:=100 -p width:=1280 -p height:=720

ros2 run sample_nodes gpu_pointcloud_publisher \
  --ros-args -p pending_ttl_ms:=250 -p publish_rate_hz:=15.0
```

Subscriber は `GpuImageSubscriber` / `GpuPointCloudSubscriber` をそれぞれ起動します。

```bash
ros2 run sample_nodes gpu_image_subscriber
ros2 run sample_nodes gpu_pointcloud_subscriber
```

## コアコンポーネントとヘルパー

- `ros2_cuda_ipc_core::ImageView` / `PointCloud2View`: CUDA IPC ハンドルを ROS 2 メッセージとして受け渡すための型アダプタ。`ros2_cuda_ipc_core/include/ros2_cuda_ipc_core` に実装があります。
- `ros2_cuda_ipc_core::BufferView`: 任意バッファ向けの基盤ビュー。Publisher/Subscriber 間で共有メモリのメタデータを運びます。
- `ros2_cuda_ipc_core::LeaseHandle`: Publisher 側でスロットの貸出状態を管理し、`pending_ttl` の経過で強制解放します。
- `sample_nodes::GpuImagePublisherHelper` / `sample_nodes::GpuPointCloudPublisherHelper`: サンプル用のユーティリティで、スロット確保・イベント送信・ビュー生成をまとめています。

---

## 開発フロー

* 設計は [doc/design.md](doc/design.md) に従う
* Issue / PR ベースで進める
* コードスタイル: clang-format, ament_lint
* CI: GitHub Actions (予定)

---

## テスト

### CI コンテナイメージのビルド

GitHub Actions の `build` ワークフローは `ghcr.io/dskkato/ros2-cuda-ipc-dev:<ROS_DISTRO>` というタグ名のコンテナイメージを前提に動作します。
`scripts/build_container.sh` は `--ros-distro` を指定すると同名のタグを自動で付与するため、各ディストロ向けのイメージを次のように作成・公開できます:

```bash
# Humble (既存の :latest を :humble にリタグする場合にも利用可能)
./scripts/build_container.sh --ros-distro humble --push

# Iron
./scripts/build_container.sh --ros-distro iron --push

# Jazzy
./scripts/build_container.sh --ros-distro jazzy --push
```

`docker login ghcr.io` を事前に実行してから `--push` を付けてください。タグを明示的に変えたい場合は `--tag` オプションで `ghcr.io/dskkato/リポジトリ:タグ` のように *プレフィックスを含めた* フルリポジトリ名を渡してください（スクリプトはこの形式を期待します）。


`ros2_cuda_ipc_core` には gtest ベースの単体テストが含まれています。

- `test_type_adapters.cpp`: CUDA IPC メッセージ型アダプタの変換確認
- `test_lease_handle.cpp`: スロット貸出しと `pending_ttl` 回収の検証

ビルドとテスト実行例:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select ros2_cuda_ipc_core sample_nodes --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select ros2_cuda_ipc_core sample_nodes
colcon test-result --verbose
```

## メモリバックエンドの選択

`ros2_cuda_ipc` は GPU メモリの共有方式として、**2 種類のバックエンド**をサポートしています。

### CUDA IPC バックエンド（デフォルト）

x86_64 + dGPU 環境で利用可能な標準的な方式です。`cudaIpcMemHandle_t` / `cudaIpcEventHandle_t` を使用してプロセス間でゼロコピー共有を行います。

**適用環境:**
- x86_64 アーキテクチャ + dGPU（NVIDIA GeForce, Quadro, Tesla など）
- CUDA IPC のメモリ共有機能がサポートされている環境

### VMM + FD バックエンド（Jetson Orin 向け）

Jetson Orin のような CUDA IPC のメモリ共有をサポートしていない環境のために用意された代替バックエンドです。CUDA Driver API の Virtual Memory Management（VMM）を使用して GPU メモリを確保し、POSIX ファイルディスクリプタ（FD）として共有します。

**適用環境:**
- Jetson Orin（統合 GPU を持つ ARM64 プラットフォーム）
- CUDA IPC のメモリ共有が使えない環境

**技術詳細:**
- Publisher が `cuMemCreate` で GPU メモリを確保し、`cuMemExportToShareableHandle` で FD 化
- FD は Unix domain socket（`/tmp/cuda_memory_pool_<uuid>.sock`）経由で配布
- Subscriber は `cuMemImportFromShareableHandle` でメモリをインポート
- イベント同期は引き続き `cudaIpcEventHandle_t` を使用

詳細な設計については [doc/design.md](doc/design.md) の「MemoryBackend の抽象化」セクションを参照してください。

### バックエンドの指定方法

バックエンドは起動時に `memory_backend` パラメータで指定します。

#### Julia Set デモでの指定例

```bash
# CUDA IPC バックエンド（デフォルト）
ros2 launch julia_set julia_set_demo.launch.py memory_backend:=cuda_ipc

# VMM + FD バックエンド（Jetson Orin 向け）
ros2 launch julia_set julia_set_demo.launch.py memory_backend:=vmm_fd
```

#### サンプルノードでの指定例

```bash
# GpuImagePublisher で VMM + FD バックエンドを使用
ros2 run sample_nodes gpu_image_publisher \
  --ros-args -p memory_backend:=vmm_fd

# GpuPointCloudPublisher で VMM + FD バックエンドを使用
ros2 run sample_nodes gpu_pointcloud_publisher \
  --ros-args -p memory_backend:=vmm_fd
```

**注意事項:**
- Publisher と Subscriber は**同じバックエンド**を使用する必要があります
- デフォルトは `cuda_ipc` です（互換性のため）
- `vmm_fd` は他の名前（`vmm-fd`, `vmm`, `fd`）でも指定可能です

## CUDA について（core は CUDA 前提）

- `ros2_cuda_ipc_core` は CUDA が利用可能な環境を前提にビルドされます（`find_package(CUDAToolkit REQUIRED)`）。
- CUDA IPC が制限された環境では gtest が自動的にスキップされる場合があります。

## ライセンス

MIT License — 詳細は [LICENSE](LICENSE) を参照してください。
