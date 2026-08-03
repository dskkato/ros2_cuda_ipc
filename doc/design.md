# GPU Zero-Copy Transport Design

この文書は、このブランチの現行実装における GPU zero-copy transport の設計と
公開 API の契約を説明する。`BufferView` を中心とした過去の設計は
[design-buffer-view-history.md](design-buffer-view-history.md) に分離している。
history 文書は現行 API の仕様ではないため、実装変更時の更新対象にしない。

## 設計目標と範囲

ROS 2 の通常のメッセージ配送を使いながら、GPU payload 自体を CPU または
Subscriber プロセスへコピーせずに共有する。メッセージには payload の識別子と
レイアウトメタデータを載せ、実際の GPU resource と block lifetime は core が管理する。

現在の transport は CUDA VMM + shareable FD を唯一の memory sharing backend とする。
- 固定 block pool と shared-memory buffer reference による publisher/subscriber 間の lifetime 管理
- C++ の raw pointer API、画像／点群の typed adapter、Python の GpuImage/DLPack adapter

CPU fallback、Python publisher、任意の ROS message の自動変換はこの設計の範囲外である。
Python adapter の詳細は [doc/python-subscriber-implementation.md](python-subscriber-implementation.md) を参照する。

## 用語と層構造

| 用語 | 役割 |
| --- | --- |
| `BufferCore` | GPU allocation と publication を識別する transport descriptor |
| `GpuImage` / `GpuPointCloud2` | `BufferCore` とアプリケーションのレイアウトメタデータを持つ ROS message |
| `BufferMapper` | `BufferCore` を import し、consumer stream に bind した read を作る mapper |
| `ReadHandle` | 一つの GPU read の所有者。imported resource と buffer reference を保持する move-only handle |
| `ImageView` / `PointCloud2View` | `ReadHandle` に画像／点群メタデータを付加する move-only typed adapter |
| buffer reference | shared-memory block の refcount を保持し、publisher による再利用を防ぐ subscriber 側の所有権 |

### Blockモデル

`GpuBufferBlock` は、1つのGPU allocation、export可能なmemory handle、CUDA
synchronization object、およびそのallocationに対応する再利用identityをひとまとまりで
表す。`BlockMetadata` は同じblockに対応するPool shared memory内の共有lifetime stateであり、
publication identity (`uid`) とrefcountを保持する。

`GpuBufferPool` はwire protocol上のオブジェクトではない。これはPublisher内部で
`GpuBufferBlock` の確保・破棄・再利用を管理するローカルなimplementation detailであり、
Subscriberから参照されることはない。Subscriberが参照するresource identityは、descriptorの
`publisher_instance_id`、`block_id`、`uid` と、対応する `BlockMetadata` で表される。

```text
Publisher process                                      Subscriber process

GpuBufferManager                                      BufferMapper
  ├─ GpuBufferPool                                    ├─ BufferMetadataCache
  │    ├─ GpuBufferBlock                              │    └─ BlockMetadata[] (mapped)
  │    │    ├─ GPU allocation                         ├─ BufferRef
  │    │    ├─ exportable memory handle                └─ ReadHandle
  │    │    └─ CUDA synchronization objects                 └─ imported GPU resource
  └─ BufferMetadataManager
       └─ Pool shared memory
            ├─ header
            └─ BlockMetadata[]

GpuBufferBlock + BlockMetadata + BufferDescriptor
                         └─ one publication/resource identity
```

`BufferDescriptor` はGPU resourceを所有せず、上記identityとimportに必要なwire情報だけを
運ぶ。Pool shared memoryはPoolごとに1つのPOSIX shared memoryを持ち、layoutは引き続き
`header` と `BlockMetadata[]` の順序で構成される。

公開 API の中心は次の関係である。

```text
ROS message
    -> BufferMapper::map(message.core, consumer_stream)
    -> optional<ReadHandle>
    -> GPU pointer + byte size

GpuImage / GpuPointCloud2
    -> ImageViewMapper / PointCloud2ViewMapper
    -> ImageView / PointCloud2View
    -> ReadHandle
```

`MappedPublication`、`BufferRef`、`BufferMetadataCache`、`IpcHandleCache`、completion
event、deferred release queue は lifetime を実装する内部要素であり、アプリケーションが
直接組み合わせる API ではない。

## ROS message と transport descriptor

### `BufferCore`

`BufferCore` は次の情報を持つ。

| field | 意味 |
| --- | --- |
| `vmm_socket_path` | VMM allocation を配布する Unix socket のパス |
| `event_handle` | producer の ready event を識別する CUDA IPC event handle |
| `shm_name` | buffer reference/refcount 用 POSIX shared memory 名 |
| `publisher_instance_id` | publisher 初期化単位の識別子 |
| `device_id` | allocation が存在する CUDA device |
| `block_id` | 固定 pool 内の block |
| `uid` | block 上の publication 世代 |
| `byte_size` | GPU buffer の論理サイズ（bytes） |

Subscriber は `publisher_instance_id`、`block_id`、`uid` を検証してから buffer reference を取得する。
同じ `block_id` でも publisher instance または uid が異なる publication は別物として扱う。

### typed message

`GpuImage` は `header`、`dtype`、`shape = {rows, cols, channels}`、byte 単位の
`strides = {row, column, channel}`、任意の `encoding`、および `BufferCore` を持つ。

`GpuPointCloud2` は `header`、`height`、`width`、`fields`、`point_step`、`row_step`、
`is_dense`、および `BufferCore` を持つ。payload は `core.byte_size` の範囲内で、少なくとも
`row_step * height` を格納できなければならない。各 field の offset とサイズは
`point_step` の範囲内でなければならない。

メッセージ定義そのものは [ros2_cuda_ipc_msgs/msg](../ros2_cuda_ipc_msgs/msg) を正とする。

## Memory sharing: VMM + FD

Publisher は CUDA VMM allocation を block ごとに作り、POSIX shareable FD を Unix domain socket
経由で配布する。`vmm_socket_path` には socket のパスを格納し、Subscriber はその socket に
接続して FD を取得し、`cuMemImportFromShareableHandle`、map、access 設定を行う。ready event
の配送は CUDA IPC event handle を使う。

VMM resource の allocation、FD server、import 済み mapping の破棄は backend と resource
cache の責務である。uid の更新だけで mapping を無条件に再作成せず、allocation または
publisher instance が変わった場合に resource identity を切り替える。

## Publisher の lifecycle

Publisher は `GpuBufferManager` と `GpuBufferPool` で block を管理する。

```text
acquire_for_publish()
    -> GPU buffer へ producer work を enqueue
    -> prepare_publish(stream)
       - descriptor を作成
       - producer ready event を record
       - reservation を commit
    -> ROS message を作成して publish
```

`prepare_publish()` が失敗した場合は descriptor を返さず、reservation は manager の
reset まで再利用しない。publish 後の middleware の成否は block の commit 状態を変えない。

固定 grace period と uid は配送保証ではない。遅延した message は、block 再利用後に
uid mismatch として破棄される可能性がある。詳細な buffer metadata protocol と publisher／
subscriber race の不変条件は [doc/buffer_metadata_protocol.md](buffer_metadata_protocol.md) を正とする。

## Subscriber の lifecycle

### `BufferMapper`

```cpp
std::optional<ros2_cuda_ipc_core::subscriber::ReadHandle> read =
    mapper.map(message.core, consumer_stream);
```

`BufferMapper::map()` は次を一つの mapping 操作として行う。

1. `publisher_instance_id` を検証する。
2. `shm_name` に対応する buffer metadata mapping を取得または attach する。
3. `block_id` と `uid` を検証し、buffer reference を取得する。
4. `IpcHandleCache` から imported resource を取得する。未登録なら VMM-FD importer で import する。
5. consumer stream に producer ready event の wait を enqueue する。
6. imported resource と buffer reference を所有する `ReadHandle` を返す。

mapping に失敗した場合、raw `BufferMapper` は `std::nullopt` を返す。詳細な理由は内部
logging に記録し、公開 API ではエラー型を返さない。

`BufferMapper` は mapper ごとに buffer metadata mapping cache を保持するため、callback ごとに作り直さず
再利用することが望ましい。IPC resource cache と buffer metadata mapping cache は process 内で resource
を再利用するが、active な `ReadHandle` の所有権とは独立している。

### `ReadHandle` の所有権と完了通知

`ReadHandle` は move-only であり、次を保持する。

- device pointer、`byte_size`、`device_id`
- imported GPU resource への参照
- buffer reference
- mapper に渡された consumer stream
- consumer read completion event

mapper は producer ready event の wait を enqueue するだけで、consumer work の完了を待たない。
アプリケーションは同じ stream に GPU work を enqueue し、work がその stream に記録されるように
しなければならない。

```text
producer ready event
    -> consumer stream wait
    -> consumer GPU work
    -> ReadHandle destruction
       - completion event を consumer stream に record
       - resource / buffer reference / event を deferred queue へ移動
    -> queue worker が順番に cuEventSynchronize
    -> handle 固有の resource と buffer reference を解放
```

したがって、consumer stream は対応する `ReadHandle` の破棄と completion event の record が
完了するまで有効でなければならない。handle を破棄した時点で直ちに block が再利用可能になる
わけではない。

### typed adapter

`ImageViewMapper` と `PointCloud2ViewMapper` は、まず `BufferMapper` で core read を作り、
ROS message のレイアウトメタデータをコピーして typed view を返す。GPU payload bytes のコピー
は行わない。

`ImageView` と `PointCloud2View` は `ReadHandle` を内包する move-only object である。raw pointer
が必要な C++ caller は `view.core.data<T>()` または `view.core.device_ptr()` を使う。

画像の通常の stream-bound mapping は次のように行う。

```cpp
auto view = image_mapper.map(message, consumer_stream);
if (!view.valid()) {
  return;
}
launch_kernel(view.core.data<uint8_t>(), consumer_stream);
```

`ImageViewMapper::map_for_dlpack()` は publication を先に取得し、DLPack の
`__dlpack__(stream)` 時に read handle と consumer stream を bind する特殊な経路である。
これは Python／DLPack adapter の ownership を framework object へ移すために必要であり、通常の
raw pointer mapping と同じ API として扱わない。

## API 契約と制約

- 公開 stream API は CUDA Driver API の `CUstream` を使う。stream の所有権は caller にあり、core は借用する。
- `ReadHandle`、`ImageView`、`PointCloud2View` はコピーできない。非同期 GPU work の完了前に所有者を破棄してはならない。
- `ReadHandle` の失敗理由は公開 API には含まれず、内部 log を確認する。
- `ImageView::valid()` は resource mapping と画像 shape の基本条件を確認する。metadata の境界検証には `sanity_check()` を使う。
- import cache と buffer metadata mapping cache は現時点で unbounded policy であり、LRU、TTL、Publisher instance 単位の自動 prune は行わない。
- Publisher process crash や長時間の Subscriber 遅延では buffer reference が残り、block が再利用できなくなる可能性がある。
- CUDA context、CUDA stream、CUDA event の lifetime は caller／core の契約に従う必要がある。

## 設計変更時の参照先

- 公開 API の概要と maintainer 向け手順: [DEVELOPING.md](../DEVELOPING.md)
- buffer reference、uid、競合安全性: [doc/buffer_metadata_protocol.md](buffer_metadata_protocol.md)
- Python／DLPack の ownership と stream semantics: [doc/python-subscriber-implementation.md](python-subscriber-implementation.md)
- 実行例: [examples/multi_process_image_fanout/README.md](../examples/multi_process_image_fanout/README.md)
