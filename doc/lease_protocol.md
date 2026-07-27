# GPU Buffer Lease Protocol

## 1. 目的と適用範囲

この文書は、`ros2_cuda_ipc` が GPU buffer slot を Publisher と Subscriber の複数
process 間で安全に再利用するための lease protocol を定義する。

この protocol は、古い message や処理中の Subscriber が参照する GPU buffer を
Publisher が上書きすることを防ぐ。ROS middleware の配送保証、Subscriber の死活監視、
Publisher 再起動後の状態引き継ぎは対象外である。

## 2. 用語

| 用語 | 定義 |
| --- | --- |
| slot | GPU buffer、ready event、共有 lease metadata の組 |
| generation | slot が publish 用に取得されるたびに増加する世代番号 |
| reservation | Publisher が保持する `slot_id` と `generation` |
| refcnt | Subscriber lease と Publisher reservation の合計参照数 |
| grace period | Publisher reservation 解放後に slot を再利用しない固定期間（100 ms） |
| lease | Subscriber が buffer を読み取り中であることを表す RAII 所有権 |
| write handle | device pointer への一時的な書き込み権。破棄時に ready event を記録する |

`pending` や ROS subscription count は lease protocol の状態として使用しない。実際に
lease を取得した Subscriber だけが `refcnt` に反映される。

## 3. 構成要素

```text
Publisher process
  GpuBufferManager
    GpuBufferPool       GPU allocation と ready event を所有
    LeaseManager        reservation と publish lifecycle を管理
      LeaseHandle       process-shared metadata を操作
    PublishSlot         1回の publish 試行を表す move-only object
      WriteHandle       CUDA stream と device pointer の scoped access

Subscriber process
  BufferViewMapper
    LeaseHandle         lease を取得
  BufferView            view の生存中 lease を保持
```

Publisher は次の順で API を使用する。

```cpp
auto slot = manager.acquire_for_publish();
{
  auto write = slot->write(stream);
  launch_gpu_work(write->data(), stream);
}
auto descriptor = slot->descriptor();
publisher->publish(make_message(*descriptor));
```

## 4. Process-shared slot state

各 slot は POSIX shared memory 上に次の metadata を持つ。

```cpp
struct SlotMeta {
  std::atomic<uint32_t> generation;
  std::atomic<uint32_t> refcnt;
  std::atomic<uint64_t> publish_timestamp_us;
};
```

shared-memory layout version は5である。attach 時には magic、layout version、capacity、
Publisher instance ID を検証する。

各atomicは対象platformでlock-freeであることを要求し、Publisherがshared memoryを作成
するときに`SlotMeta`をplacement newで構築する。Subscriberは既存のslotを再構築せずに
attachする。

`publish_timestamp_us` は `steady_clock` のマイクロ秒値で、Publisher reservation
解放時に開始した再利用抑止期間の起点を表す。0 は grace period が設定されていない
slot を表す。publish 前の早期終了にも grace period を適用するため、実際の ROS publish
時刻とは限らない。

slot の再利用条件は次である。

```text
refcnt == 0 &&
(publish_timestamp_us == 0 ||
 now_us - publish_timestamp_us >= 100000)
```

`reserved` のような別の排他 flag は持たない。Publisher が slot を取得するときに
`refcnt` を CAS で `0 -> 1` に変更し、その1件を Publisher reservation として扱う。

## 5. Publisher workflow

### 5.1 reserve

`acquire_for_publish()` は round-robin で候補を探索し、各 slot に対して次を行う。

1. `refcnt == 0` を確認する。
2. timestamp が0、または publish 開始から100 ms以上経過していることを確認する。
3. `refcnt` を CAS で `0 -> 1` に変更する。
4. CAS に失敗したら次の候補へ進む。
5. `generation` を1増加する。
6. `publish_timestamp_us` を0へ戻す。
7. reservation を返す。

Publisher reservation が存在する間は、別の Publisher が同じ slot を取得できない。
Subscriber は `refcnt != 0` だけを理由に拒否せず、generation の前後再確認で Publisher
との競合を検出する。

### 5.2 GPU work と ready event

Publisher は `slot->write(stream)` で move-only `WriteHandle` を取得し、その
`data()` から得た device pointer へ GPU work を enqueue する。WriteHandle の
destructor は同じ CUDA stream へ ready event を記録する。

WriteHandle は親 PublishSlot を借用する。PublishSlot は handle より長生きし、handle が
有効な間は move してはならない。1つの PublishSlot から取得できる WriteHandle は1回
だけである。`descriptor()` は handle の破棄と ready event 記録成功前には失敗する。
event 記録失敗は `ready_error()` から取得できる。

### 5.3 reservation release

`PublishSlot` の destructor は、ROS publish の実行有無にかかわらず次を行う。

1. reservation の generation が現在の generation と一致することを確認する。
2. `publish_timestamp_us` を現在時刻へ更新する。
3. Publisher reservation の `refcnt` を1減少させる。

Subscriber lease が残っていれば slot は再利用できない。slot の破棄は GPU work 完了、
Subscriber への配送完了、Subscriber の lease 取得完了を意味しない。

publish 前の早期 return や ready event 記録失敗でも grace period を適用する。これに
より未公開 slot にも一時的な backpressure が発生するが、明示的な commit/cancel の
呼び忘れを排除する。reservation 取得処理自体の rollback では内部 cancel を使用し、
grace period を開始しない。

## 6. Subscriber acquire と release

Subscriber は message の `shm_name`、`publisher_instance_id`、`slot_id`、`generation`
を使って lease を取得する。

acquire は次を行う。

1. shared memory へ attach し、layout と slot 範囲を検証する。
2. Publisher instance ID を検証する。
3. message の generation と slot の generation を確認する。
4. `refcnt` を CAS で1増加する。
5. generation を再確認する。
6. 再確認に失敗した場合は refcount を戻して失敗する。
7. 成功したら `LeaseHandle` を返す。

有効な `LeaseHandle` の破棄または move assignment による release は `refcnt` を1減少
させる。`BufferView` は内部で lease を保持するため、view の生存中は slot を再利用
できない。

Subscriber が import した ready event を自身の CUDA stream で待機してから buffer を
読み取ることは、Subscriber 側の契約である。

## 7. Publisher/Subscriber 競合の安全性

Publisher が先に refcount を claim した場合、Subscriber が古い generation を使って
refcount を増加できても、generation の再確認に失敗して refcount を戻す。

Subscriber が先に refcount を増加した場合、Publisher の `0 -> 1` CAS が失敗する。

```text
Publisher                              Subscriber

refcnt 0 -> 1 を CAS
generation を更新
                                        generation を確認
                                        refcnt を増加
                                        generation を再確認
                                        失敗時は refcnt を減少
publish_timestamp_us を更新
Publisher ref を解放
```

この手順により、Publisher が slot を再利用する一方で古い generation の Subscriber
lease が成立することを防ぐ。

## 8. 失敗時の挙動と既知の制約

| failure | behavior |
| --- | --- |
| reusable slot がない | Publisher の acquire が失敗する |
| GPU resource 初期化失敗 | 作成済み resource を rollback する |
| ready event 記録失敗 | descriptor を無効にし、slot 破棄時に grace period 付きで reservation を解放する |
| generation mismatch | Subscriber acquire または reservation 完了を失敗させる |
| Subscriber process crash | refcnt が残り、slot が再利用不能になる可能性がある |
| 100 ms を超える message 遅延 | slot 再利用後は generation mismatch で drop される可能性がある |

固定 grace period は配送保証ではない。publish rate、最大 Subscriber 遅延、slot 数に
応じて、遅延 message を drop し得る安全側の設計である。

`generation` は wire format と shared state の `uint32_t` を維持する。wraparound 時に
非常に古い message と一致する可能性があるため、必要なら Publisher instance と shared
memory pool を再生成する。

shared memory 上の atomic は Linux x86_64/aarch64 と対象 toolchain に依存する。現在の
実装は aligned な整数 field を process-shared atomic として扱うため、異なる ABI 間の
shared-memory 相互運用は保証しない。
