# ros2_cuda_ipc

ROS 2 CUDA IPC Zero-Copy Transport

## Primary Demo

`multi_process_image_fanout` is the main example for this repository. It
publishes one GPU RGBA image and fans it out to three independent ROS 2
subscriber processes through CUDA IPC.

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to multi_process_image_fanout
source install/setup.bash
ros2 launch multi_process_image_fanout multi_process_image_fanout.launch.xml
```

See [examples/multi_process_image_fanout/README.md](examples/multi_process_image_fanout/README.md)
and [examples/multi_process_image_fanout/doc/design.md](examples/multi_process_image_fanout/doc/design.md).

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
- `ros2_cuda_ipc_test` — CUDA IPC と VMM-FD の動作テストアプリケーション（ROS2非依存）
- `multi_process_image_fanout` — 複数プロセスへ GPU 画像をゼロコピー配信する主デモ
- `examples/legacy/gpu_image_transport` — GPU画像転送用のimage_transport プラグイン
- `examples/legacy/julia_set` — Julia集合のGPU描画デモノード

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

### Legacy Julia Set デモ実行

GPU 上で Julia 集合を描画し、`ros2_cuda_ipc_core` の `GpuLeasePool` と TypeAdapter API を使って ROS 2 プロセス間で GPU 画像を共有するデモです。
このデモは `examples/legacy` に移動済みで、既定の colcon ビルド対象からは外しています。
実行する場合は `examples/legacy/COLCON_IGNORE` を一時的に退避してから legacy package をビルドしてください。

```bash
ros2 launch julia_set julia_set_demo.launch.py
```

主なパラメータ:
- `memory_backend`: GPU メモリ共有方式（`cuda_ipc` または `vmm_fd`）
- `publish_rate_hz`: Publish 周期（既定 30 Hz）
- `slot_count`: 確保する GPU メモリスロット数（既定 4）
- `pending_ttl_ms`: 未消費スロットを強制解放する猶予時間 [ms]
- `shm_name`: lease 管理用の共有メモリ名
- `device_index`: 利用する CUDA デバイス（既定 0）
- `width`, `height`, `max_iterations`, `zoom`: 描画パラメータ

詳細は [examples/legacy/julia_set/README.md](examples/legacy/julia_set/README.md) を参照してください。

## Multi-process Image Fanout デモ実行

`multi_process_image_fanout` は、1 つの GPU 画像を複数の独立した ROS 2 プロセスへゼロコピーで配る主デモです。

```bash
ros2 launch multi_process_image_fanout multi_process_image_fanout.launch.xml
```

主なノード:
- `gpu_image_publisher`
- `preview_node`
- `encoder_like_node`
- `inference_like_node`

詳細は [examples/multi_process_image_fanout/README.md](examples/multi_process_image_fanout/README.md) と [examples/multi_process_image_fanout/doc/design.md](examples/multi_process_image_fanout/doc/design.md) を参照してください。

## コアコンポーネントとヘルパー

- `ros2_cuda_ipc_core::view::BufferView`: 任意バッファ向けの基盤ビューです。受信側では import 済み GPU resource と lease を保持します。
- `ros2_cuda_ipc_core::view::ImageView` / `ros2_cuda_ipc_core::view::PointCloud2View`: `BufferView` に画像・点群メタデータを重ねた custom type です。
- `ros2_cuda_ipc_core::mapper::BufferViewMapper` / `ros2_cuda_ipc_core::mapper::ImageViewMapper` / `ros2_cuda_ipc_core::mapper::PointCloud2ViewMapper`: `BufferCore` / `GpuImage` / `GpuPointCloud2` から View を構築する明示制御 API です。
- `ros2_cuda_ipc_core/type_adapters.hpp`: `Publisher<ros2_cuda_ipc_core::view::ImageView>` / `Subscription<ros2_cuda_ipc_core::view::ImageView>` のように View を直接扱うための薄い TypeAdapter です。内部では mapper API を呼びます。
- `ros2_cuda_ipc_core::LeaseHandle`: Publisher 側でスロットの貸出状態を管理し、`pending_ttl` の経過で強制解放します。
- `ros2_cuda_ipc_core::cuda::GpuLeasePool`: Publisher 側の GPU メモリスロット、backend 切り替え、lease 更新をまとめる共通プールです。

受信側 API は 2 系統あります。

- TypeAdapter API: `Subscription<ros2_cuda_ipc_core::view::ImageView>` / `Subscription<ros2_cuda_ipc_core::view::PointCloud2View>` をそのまま使う高レベル API
- Mapper API: `Subscription<ros2_cuda_ipc_msgs::msg::GpuImage>` などで raw message を受け、`ros2_cuda_ipc_core::mapper::ImageViewMapper::map()` で明示的に resource import する API

---

## 開発フロー

* 設計は [doc/design.md](doc/design.md) に従う
* Issue / PR ベースで進める
* コードスタイル: clang-format, ament_lint
* CI: GitHub Actions

---

## テスト

### CI コンテナイメージのビルド

GitHub Actions の `build` ワークフローは `ghcr.io/dskkato/ros2-cuda-ipc-dev:<ROS_DISTRO>` というタグ名のコンテナイメージを前提に動作します。
`scripts/build_container.sh` は `--ros-distro` を指定すると同名のタグを自動で付与するため、各ディストロ向けのイメージを次のように作成・公開できます:

```bash
# Humble (既存の :latest を :humble にリタグする場合にも利用可能)
./scripts/build_container.sh --ros-distro humble --push

# Jazzy
./scripts/build_container.sh --ros-distro jazzy --push

# Lyrical
./scripts/build_container.sh --ros-distro lyrical --push
```

`docker login ghcr.io` を事前に実行してから `--push` を付けてください。タグを明示的に変えたい場合は `--tag` オプションで `ghcr.io/dskkato/リポジトリ:タグ` のように *プレフィックスを含めた* フルリポジトリ名を渡してください（スクリプトはこの形式を期待します）。

ビルドとテスト実行例:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select ros2_cuda_ipc_core --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select ros2_cuda_ipc_core
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

**注意事項:**
- Publisher と Subscriber は**同じバックエンド**を使用する必要があります
- デフォルトは `cuda_ipc` です（互換性のため）
- `vmm_fd` は他の名前（`vmm-fd`, `vmm`, `fd`）でも指定可能です

## CUDA について（core は CUDA 前提）

- `ros2_cuda_ipc_core` は CUDA が利用可能な環境を前提にビルドされます（`find_package(CUDAToolkit REQUIRED)`）。
- CUDA IPC が制限された環境では gtest が自動的にスキップされる場合があります。

## ライセンス

MIT License — 詳細は [LICENSE](LICENSE) を参照してください。
