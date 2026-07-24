## `LeaseMappingCache` strong cache

Subscriber の `LeaseMappingCache` は、同じ Publisher instance の POSIX shared-memory mapping を
message 間で再利用するため、cache value として `std::shared_ptr<LeaseMapping>` を保持する。
callback-local な `BufferView` が破棄されても cache が mapping を所有し続けるため、定常 message で
`LeaseMapping::attach()` と `munmap()` を繰り返さない。

### Ownership

```text
LeaseMappingCache
  -> shared_ptr<LeaseMapping>
LeaseHandle
  -> shared_ptr<LeaseMapping>
BufferView
  -> shared_ptr<LeaseHandle>
```

cache entry は `shm_name` と `publisher_instance_id` の組で識別する。同じ SHM 名でも Publisher
instance が異なれば別 entry であり、Publisher restart 後の mapping を誤って再利用しない。

`LeaseMappingCache::clear()` は map を mutex から切り離してから entry を破棄する。active な
`LeaseHandle` が mapping を保持している場合、clear 後も slot metadata への参照は有効である。
最後の `LeaseHandle` が解放された時点で mapping の destructor が実行される。

cache entry は明示的な `clear()` または process 終了まで保持される。現時点では unbounded policy
であり、LRU、TTL、最大 entry 数、automatic eviction、Publisher instance 単位の prune、background
cleanup、stale process 検出は行わない。Publisher restart ごとに新しい instance entry が増える可能性がある。

### `get_or_attach()` の処理

1. `shm_name` と `publisher_instance_id` で cache を lookup する。
2. entry があれば保持中の `shared_ptr` を返す。
3. entry がなければ cache mutex の外で `LeaseMapping::attach()` を実行する。
4. mutex 内で最初に登録された mapping を entry とし、同時に生成された候補は mutex 外で破棄する。

attach failure は `nullptr` を返し、cache entry を追加しない。並列 lookup で attach が重複しても、
cache に残る entry は key ごとに一つだけで、全 caller は最終的にその entry を受け取る。

### 変更前の比較値

callback-local な `BufferView` を使った変更前の probe では、1000 message に対して次の値だった。

| counter | 値 |
|---|---:|
| `LeaseMapping::attach()` | 1000 |
| `LeaseMapping` destroy | 1000 |
| cache hit | 0 |
| Subscriber 側 `shm_open`/`openat` | 1000 |
| `fstat`/`newfstatat` | 1000 |
| `mmap` | 1000 |
| `close` | 1000 |
| `munmap` | 1000 |

### 変更後の検証結果

`test_lease_mapping_cache` の callback-local 相当テストでは、最初の lookup を含む1001回の lookupで
attach は1回、cache entry は1件だった。最初の lookup後の1000回は同じ mapping pointer を返し、各 iteration
で `LeaseHandle` を取得・解放しても loop 中の mapping destroy は0回だった。cache clear 後に destroy は1回になった。

同テストの concurrent duplicate attach では8 threadが同じ key を同時に lookupした。attach candidateは8個
生成されたが、cache entryは1件に収束し、全 threadが同じ mapping pointerを受け取った。採用されなかった7個の
candidateは mutex 外で破棄された。

`get_or_attach()` の Release build timing test（attach miss 1回、hit 10000回）は次の結果だった。

| 区間 | count | average | p50 | p95 | p99 | maximum |
|---|---:|---:|---:|---:|---:|---:|
| miss + attach | 1 | 5.950 µs | 5.950 µs | 5.950 µs | 5.950 µs | 5.950 µs |
| cache hit | 10000 | 0.0572 µs | 0.056 µs | 0.060 µs | 0.062 µs | 3.714 µs |

`BufferViewMapper::map()` 全体の専用 timing は今回の example には組み込まれていないため未計測である。

### 実example / syscall 検証

2026-07-24 に RTX 4060 Laptop GPU、CUDA IPC、4 slot、30 Hz、640x480、3 Subscriber
（preview、encoder-like、inference-like）を約40秒実行した。各 Subscriber の status log は
`received=1170` まで進み、map失敗や lease mapping attach failure は記録されなかった。

SHM名 `/ros2_cuda_ipc_lease_validation_<uuid>` に対する `strace -f` の集計は次のとおりだった。

| process | mapping `openat` | `newfstatat` | 96-byte `mmap` | mapping `close` | 96-byte `munmap` |
|---|---:|---:|---:|---:|---:|
| Publisher create | 1 (`O_CREAT`) | 0 | 1 | 1 | 1 (終了時) |
| preview Subscriber | 1 | 1 | 1 | 1 | 1 (終了時) |
| encoder-like Subscriber | 1 | 1 | 1 | 1 | 1 (終了時) |
| inference-like Subscriber | 1 | 1 | 1 | 1 | 1 (終了時) |

Subscriber側は各々、約1170 messageに対して attach相当の `openat`/`newfstatat`/`mmap`/`close` が
1回だけ発生した。cache hit/destroy counterはexample本体に未実装だが、正常に処理された1170 messageを
基準にすると各 Subscriber は attach 1、cache hit 約1169、実行中 destroy 0、終了時 destroy 1と解釈できる。
`munmap` は timeout による SIGINT 後の process cleanup で観測した。

SIGINT cleanup時にCUDA IPC resource側の `CUDA_ERROR_UNKNOWN` ログが出たが、これは既存の CUDA resource
cleanup経路の事象であり、LeaseMappingの attach再実行やSHM mapping failureではなかった。

### 関連テスト

`test_lease_mapping_cache` は次を検証する。

- callback-local 相当の lease を1000回取得・解放しても attach は1回で、cache entry は mapping を保持する。
- active lease がない状態の clear では mapping が破棄される。
- active lease がある状態の clear では mapping が生存し、lease 解放後に破棄される。
- 同一 key の concurrent duplicate attach は一つの cache entry に収束する。
- 異なる Publisher instance は同じ SHM 名でも別 entry になる。
- attach failure と attach exception は cache を汚染しない。
