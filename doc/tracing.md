# ROS 2 Tracing セットアップ手順 (Ubuntu 22.04)

Ubuntu 22.04 上で ROS 2 Humble / Iron のトレースを取得するための手順です。
トレーサは [LTTng](https://lttng.org/) と [ros2_tracing](https://github.com/ros2/ros2_tracing) を利用します。

## 前提パッケージのインストール

### LTTng 関連ツール

```bash
sudo apt update
sudo apt install \
  lttng-tools lttng-modules-dkms liblttng-ust-dev babeltrace2
```

- `lttng-tools`: トレースセッション作成・制御コマンド (`lttng`) を提供します。
- `lttng-modules-dkms`: カーネルトレーサの DKMS モジュールを追加します。
- `liblttng-ust-dev`: ユーザー空間トレースを有効にするためのライブラリです。
- `babeltrace2`: 収集したトレースを解析・閲覧するためのツールです。

カーネルモジュールをロードして有効化します（再起動後も必要になる場合があります）。

```bash
sudo modprobe lttng-tracer
```

### ros2_tracing パッケージ

ROS 2 ディストリビューションに対応したパッケージをインストールしてください。

```bash
ROS_DISTRO=humble  # Iron の場合は "iron"
sudo apt install \
  ros-${ROS_DISTRO}-ros2trace \
  ros-${ROS_DISTRO}-ros2trace-analysis \
  ros-${ROS_DISTRO}-tracetools \
  ros-${ROS_DISTRO}-tracetools-readme
```

> `ROS_DISTRO` 環境変数はご利用のディストロ名 (`humble` または `iron`) に置き換えてください。

## ROS 2 環境の読み込み

トレース CLI (`ros2 trace`) は ROS 2 のセットアップを読み込んだシェルで実行する必要があります。

```bash
source /opt/ros/<distro>/setup.bash
# 例: source /opt/ros/humble/setup.bash
```

## ワークスペースのビルド（トレース有効）

`tracetools` で計装されたパッケージをトレース可能にするには、CMake オプション `TRACETOOLS_DISABLED=OFF` を指定してビルドします。
本リポジトリでは追加の CMake オプションは不要ですが、以下のように指定するとトレースポイントが確実に有効化されます。

```bash
# ROS 2 環境を読み込み
source /opt/ros/<distro>/setup.bash

# ワークスペースルートでビルド
colcon build --symlink-install \
  --cmake-args -DTRACETOOLS_DISABLED=OFF

# ビルド成果を反映
source install/setup.bash
```

## トレース CLI の確認

`ros2 trace` が利用可能か、トレースプロバイダが認識されているかを確認します。

```bash
ros2 trace --list
```

LTTng セッションデーモンが起動・カーネルモジュールがロードされていれば、利用可能なトレースポイント一覧が表示されます。
必要に応じて `lttng-sessiond` を起動したり、上記の `modprobe` を再実行してください。
