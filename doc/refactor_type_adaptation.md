# Type Adaptation Refactor Plan

## 目的

ROS 2 の TypeAdapter 互換性を維持しつつ、GPU resource mapping を独立した明示的な API として切り出す。

現在の受信側 TypeAdapter は `ros2_cuda_ipc_core/include/ros2_cuda_ipc_core/type_adapters.hpp` と `detail/type_adapters.hpp` に実装されている。特に `detail/type_adapters.hpp` は CUDA Runtime API、CUDA Driver API、POSIX socket、libuuid、cache、VMM map まで含むため、TypeAdapter の変更影響範囲が広く、backend 追加、resource lifetime 修正、明示的なエラー処理、単体テストが難しい。

このリファクタ後の TypeAdapter は、ROS 2 の入口として既存 API を保つ薄い wrapper とする。resource mapping の本体は Mapper/Importer API に移す。

## 現状

主な実装箇所:

- `ros2_cuda_ipc_core/include/ros2_cuda_ipc_core/type_adapters.hpp`
- `ros2_cuda_ipc_core/include/ros2_cuda_ipc_core/detail/type_adapters.hpp`

現在の受信側フロー:

```text
GpuImage / BufferCore ROS msg
  -> rclcpp::TypeAdapter::convert_to_custom()
  -> LeaseHandle::acquire()
  -> BufferCore.backend を MemoryBackendKind に変換
  -> cudaIpcOpenEventHandle()
  -> cudaIpcOpenMemHandle() or VMM-FD import
  -> process-wide IPC handle cache 参照/更新
  -> BufferView / ImageView / PointCloud2View を返す
```

現在の TypeAdapter が担っている責務:

- ROS message と custom type のフィールド変換
- `BufferCore.backend` の解釈
- `LeaseHandle` の取得と失敗時の無効 View 化
- CUDA IPC event/memory handle の open
- VMM-FD payload の parse
- Unix domain socket への接続と FD 受信
- `cuInit`、`cuMemImportFromShareableHandle`、VMM map
- import 済み handle/mapping の process-wide cache
- cache race 時の重複 open/import の後始末
- ログ出力と一部のエラー方針

## 方針

TypeAdapter は互換性と利便性のために残す。ただし、TypeAdapter の責務は「ROS 2 の入口」と「Mapper への委譲」に限定し、GPU resource mapping は明示的な Mapper/Importer API に委譲する。

最終的には次の2系統の API を提供する。

- 高レベル API: `rclcpp::Publisher<ImageView>` / `rclcpp::Subscription<ImageView>` を使う TypeAdapter 経由の API
- 明示制御 API: `ros2_cuda_ipc_msgs::msg::GpuImage` を subscribe し、ユーザーが `ImageViewMapper::map()` を呼ぶ API

TypeAdapter は高レベル API の薄い wrapper として、内部で明示制御 API を呼ぶ。

## 目標構造

```text
TypeAdapter
  -> default ImageViewMapper / PointCloud2ViewMapper / BufferViewMapper
    -> IpcHandleCache
    -> BackendImporterRegistry
      -> CudaIpcImporter
      -> VmmFdImporter
```

各コンポーネントの責務:

- `TypeAdapter`: ROS 2 custom type の入口。自前の resource 処理は持たず Mapper に委譲する。
- `BufferViewMapper`: `BufferCore` から `BufferView` を作る中心。lease 取得、backend importer 呼び出し、失敗時の無効 View 化を担う。
- `ImageViewMapper`: `GpuImage` message の画像 metadata を `ImageView` にコピーし、`BufferViewMapper` の結果と組み合わせる。
- `PointCloud2ViewMapper`: `GpuPointCloud2` message の点群 metadata を `PointCloud2View` にコピーし、`BufferViewMapper` の結果と組み合わせる。
- `IpcHandleCache`: import 済み CUDA IPC/VMM resources の process-wide cache。key/hash、mutex、重複 import の後始末を閉じ込める。
- `BackendImporter`: backend ごとの import interface。
- `CudaIpcImporter`: `cudaIpcOpenMemHandle` と `cudaIpcOpenEventHandle` を扱う。
- `VmmFdImporter`: VMM payload parse、socket 接続、FD 受信、`cuMemImportFromShareableHandle`、VMM map を扱う。

`ImageViewMapper` と `PointCloud2ViewMapper` は `BufferViewMapper` を所有または参照する。cache は Mapper ごとに個別所有せず、既定では process-wide singleton を共有する。

## ファイル配置

新設する public headers:

- `ros2_cuda_ipc_core/include/ros2_cuda_ipc_core/buffer_view_mapper.hpp`
- `ros2_cuda_ipc_core/include/ros2_cuda_ipc_core/image_view_mapper.hpp`
- `ros2_cuda_ipc_core/include/ros2_cuda_ipc_core/pointcloud2_view_mapper.hpp`
- `ros2_cuda_ipc_core/include/ros2_cuda_ipc_core/ipc_handle_cache.hpp`

新設する internal headers:

- `ros2_cuda_ipc_core/include/ros2_cuda_ipc_core/detail/backend_importer.hpp`
- `ros2_cuda_ipc_core/include/ros2_cuda_ipc_core/detail/cuda_ipc_importer.hpp`
- `ros2_cuda_ipc_core/include/ros2_cuda_ipc_core/detail/vmm_fd_importer.hpp`

新設する source files:

- `ros2_cuda_ipc_core/src/buffer_view_mapper.cpp`
- `ros2_cuda_ipc_core/src/image_view_mapper.cpp`
- `ros2_cuda_ipc_core/src/pointcloud2_view_mapper.cpp`
- `ros2_cuda_ipc_core/src/ipc_handle_cache.cpp`
- `ros2_cuda_ipc_core/src/detail/cuda_ipc_importer.cpp`
- `ros2_cuda_ipc_core/src/detail/vmm_fd_importer.cpp`

`type_adapters.hpp` は引き続き public header とするが、CUDA/POSIX/VMM の詳細 include を直接持たない状態を目標にする。`detail/type_adapters.hpp` は最終的に削除するか、互換用の最小 helper だけに縮小する。

## Public API

### BufferViewMapper

```cpp
namespace ros2_cuda_ipc_core {

struct BufferViewMapperOptions {
  rclcpp::Logger logger = rclcpp::get_logger("ros2_cuda_ipc_core.BufferViewMapper");
};

class BufferViewMapper {
 public:
  explicit BufferViewMapper(BufferViewMapperOptions options = {});

  BufferView map(const ros2_cuda_ipc_msgs::msg::BufferCore& msg) const;
};

BufferView map_buffer_view(const ros2_cuda_ipc_msgs::msg::BufferCore& msg);

}  // namespace ros2_cuda_ipc_core
```

`map()` は失敗時に例外を投げず、無効な `BufferView` を返す。`map_buffer_view()` は既定 mapper を使う convenience function とする。

### ImageViewMapper

```cpp
namespace ros2_cuda_ipc_core {

class ImageViewMapper {
 public:
  explicit ImageViewMapper(BufferViewMapper buffer_mapper = BufferViewMapper{});

  ImageView map(const ros2_cuda_ipc_msgs::msg::GpuImage& msg) const;
};

ImageView map_image_view(const ros2_cuda_ipc_msgs::msg::GpuImage& msg);

}  // namespace ros2_cuda_ipc_core
```

### PointCloud2ViewMapper

```cpp
namespace ros2_cuda_ipc_core {

class PointCloud2ViewMapper {
 public:
  explicit PointCloud2ViewMapper(BufferViewMapper buffer_mapper = BufferViewMapper{});

  PointCloud2View map(const ros2_cuda_ipc_msgs::msg::GpuPointCloud2& msg) const;
};

PointCloud2View map_pointcloud2_view(
    const ros2_cuda_ipc_msgs::msg::GpuPointCloud2& msg);

}  // namespace ros2_cuda_ipc_core
```

### 送信側変換 helper

送信側 TypeAdapter の `convert_to_ros_message()` は helper に委譲する。

```cpp
namespace ros2_cuda_ipc_core {

void fill_buffer_core_message(
    const BufferView& view,
    ros2_cuda_ipc_msgs::msg::BufferCore& msg);

void fill_gpu_image_message(
    const ImageView& view,
    ros2_cuda_ipc_msgs::msg::GpuImage& msg);

void fill_gpu_pointcloud2_message(
    const PointCloud2View& view,
    ros2_cuda_ipc_msgs::msg::GpuPointCloud2& msg);

}  // namespace ros2_cuda_ipc_core
```

既存の `fill_buffer_core_message()` は維持し、`type_adapters.hpp` から mapper/helper 用 header へ移す。関数名は既存互換を優先し、破壊的 rename はしない。

## Internal API

### ImportResult

```cpp
namespace ros2_cuda_ipc_core::detail {

struct ImportedBuffer {
  void* dev_ptr = nullptr;
  cudaEvent_t event = nullptr;
  CUdeviceptr vmm_address = 0;
  CUmemGenericAllocationHandle vmm_allocation = 0;
  std::size_t allocation_size = 0;
};

}  // namespace ros2_cuda_ipc_core::detail
```

`ImportedBuffer` は cache entry と importer の戻り値で共有する。CUDA IPC では `vmm_*` はゼロのままとする。

### BackendImporter

```cpp
namespace ros2_cuda_ipc_core::detail {

class BackendImporter {
 public:
  virtual ~BackendImporter() = default;

  virtual std::optional<ImportedBuffer> import(
      const ros2_cuda_ipc_msgs::msg::BufferCore& msg,
      const cudaIpcEventHandle_t& event_handle,
      const rclcpp::Logger& logger) const = 0;
};

}  // namespace ros2_cuda_ipc_core::detail
```

Importer は lease を扱わない。lease の取得と `BufferView` への格納は `BufferViewMapper` の責務とする。

### IpcHandleCache

```cpp
namespace ros2_cuda_ipc_core {

struct IpcHandleKey {
  uint8_t backend = 0;
  MemoryHandlePayload mem{};
  std::array<uint8_t, sizeof(cudaIpcEventHandle_t)> event{};
};

class IpcHandleCache {
 public:
  static IpcHandleCache& instance();

  std::optional<detail::ImportedBuffer> find(const IpcHandleKey& key);

  detail::ImportedBuffer insert_or_discard_duplicate(
      const IpcHandleKey& key,
      detail::ImportedBuffer imported);

  std::size_t size() const;

 private:
  // mutex + unordered_map
};

}  // namespace ros2_cuda_ipc_core
```

`insert_or_discard_duplicate()` は現行実装の race 処理を閉じ込める。別 thread が先に同じ key を insert 済みなら、引数の一時 resource を破棄し、既存 entry を返す。

一時 resource の破棄ルール:

- CUDA IPC: `cudaIpcCloseMemHandle(dev_ptr)` と `cudaEventDestroy(event)`
- VMM-FD: `cuMemUnmap`、`cuMemAddressFree`、`cuMemRelease`、`cudaEventDestroy(event)`

cache entry 自体は subscriber process lifetime まで保持する。現行挙動と同じく、cache clear は初回実装の対象外とする。

## 受信側詳細フロー

`BufferViewMapper::map()`:

```text
1. msg.backend を MemoryBackendKind に変換する
2. LeaseHandle::acquire(msg.shm_name, msg.slot_id, msg.generation)
3. lease 失敗なら warn log を出し、無効 BufferView を返す
4. cudaIpcEventHandle_t を msg.event_handle から復元する
5. IpcHandleKey を作る
6. IpcHandleCache::find(key) を見る
7. cache hit なら cached dev_ptr/event を使う
8. cache miss なら backend importer で import する
9. import 成功なら IpcHandleCache::insert_or_discard_duplicate() に渡す
10. BufferView に dev_ptr/event/metadata/lease/handles を詰めて返す
```

`ImageViewMapper::map()`:

```text
1. BufferViewMapper::map(msg.core)
2. core が invalid なら default ImageView を返す
3. dtype/shape/strides/encoding/header をコピーする
4. core と metadata を持つ ImageView を返す
```

`PointCloud2ViewMapper::map()`:

```text
1. BufferViewMapper::map(msg.core)
2. header は core invalid の場合でもコピーしてよい
3. core が invalid なら core invalid の PointCloud2View を返す
4. height/width/point_step/row_step/is_dense/fields をコピーする
5. core と metadata を持つ PointCloud2View を返す
```

既存実装では `PointCloud2View` は core invalid 時にも header を返している。この挙動は維持する。`ImageView` は core invalid 時に default を返しており、この挙動も互換性のため維持する。

## 明示制御 API の利用例

利用者が TypeAdapter を使わない場合の基本形:

```cpp
class ConsumerNode : public rclcpp::Node {
 public:
  ConsumerNode()
      : Node("consumer"),
        image_mapper_(make_image_mapper(get_logger())) {
    subscription_ =
        create_subscription<ros2_cuda_ipc_msgs::msg::GpuImage>(
            "image_gpu", rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
            [this](const ros2_cuda_ipc_msgs::msg::GpuImage& msg) {
              auto view = image_mapper_.map(msg);
              if (!view.core.valid()) {
                return;
              }
              view.enqueue_ready_event(stream_);
              // GPU processing...
            });
  }

 private:
  static ros2_cuda_ipc_core::ImageViewMapper make_image_mapper(
      const rclcpp::Logger& logger) {
    ros2_cuda_ipc_core::BufferViewMapperOptions options;
    options.logger = logger;
    return ros2_cuda_ipc_core::ImageViewMapper(
        ros2_cuda_ipc_core::BufferViewMapper(options));
  }

  ros2_cuda_ipc_core::ImageViewMapper image_mapper_;
  rclcpp::Subscription<ros2_cuda_ipc_msgs::msg::GpuImage>::SharedPtr
      subscription_;
};
```

この API では、GPU resource を開く処理が `map()` 呼び出しとして明示される。失敗時は例外ではなく無効 View を返す方針を維持する。

## TypeAdapter API

TypeAdapter を使う場合の利用コードは従来通りに保つ。

```cpp
subscription_ = create_subscription<ros2_cuda_ipc_core::ImageView>(
    "image_gpu", qos,
    [this](const ros2_cuda_ipc_core::ImageView& view) {
      if (!view.core.valid()) {
        return;
      }
      view.enqueue_ready_event(stream_);
    });
```

TypeAdapter 内部は Mapper 呼び出しだけに薄くする。

```cpp
static void convert_to_custom(const ros_message_type& msg,
                              custom_type& view) {
  view = ros2_cuda_ipc_core::map_image_view(msg);
}
```

`map_image_view()` は process-wide cache を共有する既定 mapper を使う。カスタム logger や将来の cache lifetime 制御をしたい場合は、TypeAdapter ではなく明示制御 API を使う。

## エラー方針

既存方針と同じく、受信側 map/import 失敗は例外にしない。

- lease 取得失敗: warn log を出し、無効 View を返す
- backend 未対応: warn log を出し、無効 View を返す
- CUDA IPC event open 失敗: warn log を出し、無効 View を返す
- CUDA IPC memory open 失敗: warn log を出し、無効 View を返す
- VMM-FD payload parse 失敗: warn log を出し、無効 View を返す
- VMM-FD socket/FD/import/map 失敗: warn log を出し、無効 View を返す
- resource import 途中で失敗した場合: 取得済み lease と一時 resource を確実に解放する

ログは Mapper/Importer 側で出す。TypeAdapter は原則として追加ログを出さない。これにより TypeAdapter と明示制御 API で同じ失敗挙動になる。

`backend_from_byte()` は現在 unknown backend を `CUDA_IPC` に丸める。リファクタでは `BufferViewMapper` 側で `msg.backend` の raw value を先に検査し、`CUDA_IPC` と `VMM_FD` 以外は unsupported として無効 View を返す。`backend_from_byte()` の既存挙動は互換性のため変更しない。

## キャッシュ方針

現在の process-wide cache は維持する。ただし、実装は `IpcHandleCache` として独立させる。

初期方針:

- cache key は backend、memory payload、event handle を含める
- cache entry は dev ptr、event、VMM address/allocation/size を保持する
- cache entry は subscriber process lifetime まで保持する
- publisher lifecycle を完全には追跡しない制限を public doc に明記する
- `clear()` / `erase(key)` は初回実装では提供しない

将来の拡張:

- explicit `clear()` / `erase(key)` API
- publisher restart 検出時の cache invalidation
- VMM-FD socket owner 検証
- resource leak inspection 用の debug counter

## テスト方針

既存の `test_type_adapters.cpp` は Mapper 単位のテストへ分割する。

追加/更新するテスト:

- `test_buffer_view_mapper.cpp`
  - lease failure returns invalid
  - unsupported backend returns invalid
  - VMM-FD invalid payload returns invalid
  - VMM-FD socket missing returns invalid and lease refcount returns to zero
  - `fill_buffer_core_message()` copies CUDA IPC handles
  - `fill_buffer_core_message()` preserves VMM payload
- `test_image_view_mapper.cpp`
  - core invalid returns invalid/default ImageView
  - metadata is copied when core is valid
  - `fill_gpu_image_message()` copies metadata and core
- `test_pointcloud2_view_mapper.cpp`
  - core invalid preserves existing PointCloud2 invalid behavior
  - fields and layout are copied when core is valid
  - `fill_gpu_pointcloud2_message()` copies fields and core
- `test_ipc_handle_cache.cpp`
  - key equality/hash includes backend, mem payload, event handle
  - duplicate insert returns existing entry
  - duplicate insert destroys temporary resource through injectable cleanup hook or a small test-only fake entry
- `test_type_adapters.cpp`
  - TypeAdapter delegates receive path to Mapper observable behavior
  - TypeAdapter send path delegates to fill helpers

CUDA IPC success tests may remain disabled or skipped when the environment does not support CUDA IPC. Non-CUDA metadata tests should not require a live GPU.

## 移行ステップ

1. `IpcHandleCache` と `ImportedBuffer` を新設し、key/hash/cache entry/mutex を `detail/type_adapters.hpp` から移す。
2. `CudaIpcImporter` と `VmmFdImporter` を新設し、backend ごとの open/import 処理を分離する。
3. `BufferViewMapper` を新設し、`BufferCore` から `BufferView` を作る処理を TypeAdapter から移す。
4. `ImageViewMapper` と `PointCloud2ViewMapper` を新設し、metadata 変換をまとめる。
5. `fill_gpu_image_message()` と `fill_gpu_pointcloud2_message()` を追加し、送信側変換も helper に委譲する。
6. TypeAdapter は Mapper/helper 呼び出しだけに薄くする。
7. `detail/type_adapters.hpp` から CUDA/POSIX/VMM の詳細を削除する。不要になればファイル自体を削除する。
8. 既存テストを Mapper 単位に分割し、TypeAdapter テストは wrapper として最小限にする。
9. `README.md` に「TypeAdapter API」と「明示制御 Mapper API」の2系統を記載する。
10. `doc/design.md` と `doc/lease_handle.md` の TypeAdapter 記述を、新構造に合わせて更新する。

## 互換性条件

このリファクタで壊してはいけないこと:

- `#include "ros2_cuda_ipc_core/type_adapters.hpp"` は引き続き有効
- `rclcpp::Publisher<BufferView>` / `Subscription<BufferView>` は引き続き有効
- `rclcpp::Publisher<ImageView>` / `Subscription<ImageView>` は引き続き有効
- `rclcpp::Publisher<PointCloud2View>` / `Subscription<PointCloud2View>` は引き続き有効
- `RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(...)` は維持
- `fill_buffer_core_message()` の名前と挙動は維持
- map/import 失敗時に例外を投げない
- process-wide cache による repeated open/import 抑制を維持

## 完了条件

実装完了の判定条件:

- `type_adapters.hpp` から lease/import/cache/socket/VMM の直接実装が消えている
- TypeAdapter の `convert_to_custom()` は Mapper を呼ぶだけになっている
- TypeAdapter の `convert_to_ros_message()` は fill helper を呼ぶだけになっている
- Mapper API を使って TypeAdapter なしで `GpuImage` を `ImageView` に map できる
- Mapper API を使って TypeAdapter なしで `GpuPointCloud2` を `PointCloud2View` に map できる
- `colcon test --packages-select ros2_cuda_ipc_core` が通るか、CUDA IPC 環境依存テストのみ skip/disabled である
- README と design docs に新しい API 境界が反映されている

## 非目標

このリファクタでは、以下は直接の対象にしない。

- ROS message 定義の変更
- `LeaseHandle` の SHM layout 変更
- `GpuLeasePool` の publisher-side slot 管理方針変更
- publisher restart の完全検出
- VMM-FD socket owner 検証の実装
- cache `clear()` / `erase(key)` の実装
- PointCloud サンプル復活
