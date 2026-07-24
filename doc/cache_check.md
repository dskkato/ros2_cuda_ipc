## 結論

`IpcHandleCache` は import 済み `ImportedResources` を `shared_ptr` で strong ownership し、
callback 内だけで `BufferView` を保持する場合も同じ key の resource を message 間で再利用する。
cache entry は `clear()` または process 終了まで保持され、active な `BufferView` は cache clear 後も
同じ resource の shared ownership により有効である。

以下の Scenario 1 は strong cache 化前の比較値、Scenario 2 は strong owner を使った合成検証である。
strong cache 化後の実GPU結果は「実装後の検証」に記録する。

## 実行条件

| 項目 | 値 |
|---|---|
| commit | `facdd2c5aaaae0ee268b702ad38acefcf72dcefd` |
| GPU | NVIDIA GeForce RTX 4060 Laptop GPU |
| Driver / CUDA | 580.173.02 / CUDA 13.0 |
| build | Release, `-O3 -DNDEBUG` |
| backend | CUDA IPC |
| slot数 | 4 |
| publish rate | 30 Hz |
| Subscriber数 | 3 |
| message数 | 各Subscriber 1124 map |
| warm-up | 100 message |

## 変更前の比較値: callback内だけBufferViewを保持

一時的なCUDA Driver API interposerで、実際のCUDA IPC open/closeを計測しました。

### Subscriberごとの結果

| counter | 値 |
|---|---:|
| map_count | 1124 |
| cache_hit_count | 0 |
| cache_miss_count | 1124 |
| memory_import_count | 1124 |
| event_import_count | 1124 |
| imported_resource_create_count | 1124 |
| imported_resource_release_count | 1124 |
| duplicate_import_count | 0 |
| cache_entry_count | 0 |

warm-up後の1024 mapはすべて cache miss による再importでした。

### slot別集計

| slot | map | hit | import | release |
|---:|---:|---:|---:|---:|
| 0 | 281 | 0 | 281 | 281 |
| 1 | 281 | 0 | 281 | 281 |
| 2 | 281 | 0 | 281 | 281 |
| 3 | 281 | 0 | 281 | 281 |

つまり、同じslotに戻る間隔は約4 messageですが、resourceは再利用されず、毎回importされています。

### `map()` latency

warm-up後の再import：

| Subscriber | average | p50 | p95 | p99 | maximum |
|---|---:|---:|---:|---:|---:|
| encoder-like | 0.896 ms | 0.923 ms | 1.360 ms | 2.035 ms | 2.515 ms |
| inference-like | 0.880 ms | 0.892 ms | 1.333 ms | 2.106 ms | 2.461 ms |

初回CUDA初期化を含むcold importでは、最初の4 resourceについて平均約59.5 ms、最大約237.8 msでした。これは定常処理から分離しています。

preview Subscriberも、別計測で各slot 281回のmemory/event importとreleaseを確認しました。

## 比較検証: slotごとにviewをstrong保持

一時的なmapper/cache harnessで確認しました。

| counter | 値 |
|---|---:|
| map_count | 1000 |
| cache_hit_count | 1000 |
| cache_miss_count | 0 |
| 追加memory import | 0 |
| 追加event import | 0 |
| resource release | 最後に4回 |
| cache_entry_max | 4 |
| final cache_entry_count | 0 |

各slotのviewを保持している間は、既存の `ImportedResources` を再利用できています。

cache-hit時のCPU側map latencyは約0.271 µsでした。ただし、これはCUDA APIを合成したcache/lifetime検証であり、実GPUのhit latencyではありません。実exampleではhit自体が発生しませんでした。

## Strong cache の ownership

実装は次の構造です。

- [`IpcHandleCache`](</home/dskkato/workspace/ros2_cuda_ipc/ros2_cuda_ipc_core/include/ros2_cuda_ipc_core/subscriber/ipc_handle_cache.hpp:38>) は
  `shared_ptr<const ImportedResources>` を map に保持
- [`BufferViewMapper::map()`](</home/dskkato/workspace/ros2_cuda_ipc/ros2_cuda_ipc_core/src/subscriber/buffer_view_mapper.cpp:124>) は
  cache hit なら既存 entry を使い、miss のときだけ importer を呼ぶ
- [`BufferView`](</home/dskkato/workspace/ros2_cuda_ipc/ros2_cuda_ipc_core/src/subscriber/buffer_view.cpp:98>) も同じ
  resource の shared ownership を保持
- `clear()` 後も view が保持されていれば resource は生存し、最後の cache/view reference 解放時に
  `cuIpcCloseMemHandle` と `cuEventDestroy` が呼ばれる

exampleのcallbackは次の形です。

```cpp
auto view = map_image_view(message);
on_image(view);
```

callback終了時に `view` が破棄されても、cache が `ImportedResources` を保持するため、次の message で
同じ key の entry を取得できます。

変更前の比較計測ではmemory import、event import、resource releaseがすべて同数でした。

## Resource identity

実測したkeyは、publisher instance、backend、device、slot、opaque handle digestで識別しました。

例:

| slot | memory handle digest | event handle digest | publisher instance |
|---:|---|---|---|
| 0 | `9800c7e1d5302041` | `fcda49341249e6ff` | `e129aa9f74e94ce2a5cb30e2e1bdca76` |
| 1 | `89873a1d813e25f0` | `50267dbf2231455e` | 同上 |
| 2 | `5827042d90071a6f` | `8d4f6a71dbcff2e1` | 同上 |
| 3 | `6aab271c8f5d8672` | `e09b9efcebb75140` | 同上 |

変更前は各 resource が281回 import、281回 releaseされました。

## 実装後の設計

| 項目 | 方針 |
|---|---|
| ownership | `IpcHandleCache` と `BufferView` が同じ `shared_ptr<const ImportedResources>` を共有 |
| hit | 同じ publisher instance/backend/memory handle/event handle の key なら既存 entry を返す |
| clear | map を mutex 外で破棄し、active view の resource は保持 |
| policy | unbounded。上限、LRU、TTL、automatic eviction、instance 単位 prune は導入しない |
| restart | publisher instance identity が key に含まれるため、restart 後は異なる entry が追加され得る |

## 実装後の検証

2026-07-24 に RTX 4060 Laptop GPU（driver 580.173.02、CUDA 13.0）で、Release build の
`multi_process_image_fanout` を `cuda_ipc`、4 slot、30 Hz、640x480、約50秒で実行した。
各 Subscriber は約1400 messageを callback 内だけで処理した。

| counter | encoder-like | inference-like |
|---|---:|---:|
| map_count | 約1400（status log） | 約1400（status log） |
| memory_import_count | 4 | 4 |
| event_import_count | 4 | 4 |
| imported_resource_create_count | 4 | 4 |
| messageごとの追加import | 0 | 0 |
| 実行中のresource release | 0 | 0 |

4つのresource identityがそれぞれ一度だけ importされたため、slotごとの import は 1回、
残り約1396 messageは cache hit と判断できる。preview nodeも約1400 messageを処理し、同じ
4 resourceだけを使用した。

`BufferViewMapper::map()` の厳密な実GPU latencyは、exampleが `ImageViewMapper` の内部呼び出しを
使うため、今回の一時 Driver API interposerでは直接取得できなかった。したがって、0.9 msの
変更前値に対する変更後の厳密な平均値は未計測である。一方、memory/event importが初回4回で
止まったことから、定常messageでのIPC importコストが発生していないことは確認できた。

終了時はSIGINTによる停止を使ったため、SubscriberのCUDA context cleanupで
`CUDA_ERROR_UNKNOWN` が記録され、process終了時のDriver API release回数はこの計測では確定できなかった。
unit testでは `clear()` 後に active view が保持する場合の release=0、最後の view解放後の release=1、
active viewなしの `clear()` 後の release=1 を確認している。

## 使用した一時計測物

- [実GPU用interposer](/tmp/ros2_cuda_ipc_count_interposer.cpp)
- Scenario 1/2 の比較 harness
- 合成 CUDA shim
- [実GPU計測ログ](/tmp/ros2_cuda_ipc_actual_measure2.log)

core Release build、core 81 tests、example Release build は成功した。
