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
````

### Python バインディング

Python バインディングは `ros2_cuda_ipc_py` としてビルドされ、
`import ros2_cuda_ipc_py` で利用可能です。
DLPack ブリッジを通じて PyTorch / CuPy 等にゼロコピーで渡すことができます。

### サンプルノード実行（予定）

```bash
# Publisher (疑似カメラ + YUV->BGR)
ros2 run ros2_cuda_ipc sample_publisher

# Subscriber (プレビュー縮小)
ros2 run ros2_cuda_ipc sample_preview

# Subscriber (エンコーダ)
ros2 run ros2_cuda_ipc sample_encoder
```

---

## 開発フロー

* 設計は [doc/design.md](doc/design.md) に従う
* Issue / PR ベースで進める
* コードスタイル: clang-format, ament\_lint
* CI: GitHub Actions (予定)

---

## ライセンス

Apache License 2.0

