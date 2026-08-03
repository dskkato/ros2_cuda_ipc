# GPU Buffer Metadata Protocol

## 1. 目的と適用範囲

この文書は、`ros2_cuda_ipc` が GPU buffer block を Publisher と Subscriber の複数
process 間で安全に再利用するための buffer metadata protocol を定義する。

この protocol は、古い message や処理中の Subscriber が参照する GPU buffer を
Publisher が上書きすることを防ぐ。ROS middleware の配送保証、Subscriber の死活監視、
Publisher 再起動後の状態引き継ぎは対象外である。

## 2. 用語

| 用語 | 定義 |
| --- | --- |
| block | GPU buffer、ready event、共有 buffer reference metadata の組 |
| uid | block が publish 用に取得されるたびに増加する世代番号 |
| reservation | Publisher が保持する `block_id` と `uid` |
| refcount | Subscriber buffer reference と Publisher reservation の合計参照数 |
| grace period | publish 後に block を再利用しない固定期間（100 ms） |
| buffer reference | Subscriber が buffer を読み取り中であることを表す RAII 所有権 |

`pending` や ROS subscription count は buffer metadata protocol の状態として使用しない。実際に
buffer reference を取得した Subscriber だけが `refcount` に反映される。

## 3. 構成要素

```text
Publisher process
  GpuBufferManager
    GpuBufferPool       GPU allocation と ready event を所有
    BufferMetadataManager        reservation と publish lifecycle を管理
      BufferRef       process-shared metadata を操作
    PublishBlock         1回の publish 試行を表す move-only object

Subscriber process
  BufferMapper
    ReadHandle          imported resource と buffer reference を保持
                        producer ready event を consumer stream で待機
```

Publisher は次の順で API を使用する。

```cpp
auto block = manager.acquire_for_publish();
launch_gpu_work(block->device_ptr(), stream);
auto descriptor = block->prepare_publish(stream);
if (!descriptor) {
  return;
}
publisher->publish(make_message(descriptor.value()));
```

## 4. Process-shared block state

各 block は POSIX shared memory 上に次の metadata を持つ。

```cpp
struct BlockMetadata {
  std::atomic<uint32_t> uid;
  std::atomic<uint32_t> refcount;
  std::atomic<uint64_t> publish_timestamp_us;
};
```

shared-memory layout version は5である。attach 時には magic、layout version、capacity、
Publisher instance ID を検証する。

各atomicは対象platformでlock-freeであることを要求し、Publisherがshared memoryを作成
するときに`BlockMetadata`をplacement newで構築する。Subscriberは既存のblockを再構築せずに
attachする。

`publish_timestamp_us` は `steady_clock` のマイクロ秒値で、最後に成功した
`prepare_publish()` の commit 時刻を表す。0 はまだ commit されていない block を表す。

block の再利用条件は次である。

```text
refcount == 0 &&
(publish_timestamp_us == 0 ||
 now_us - publish_timestamp_us >= 100000)
```

`reserved` のような別の排他 flag は持たない。Publisher が block を取得するときに
`refcount` を CAS で `0 -> 1` に変更し、その1件を Publisher reservation として扱う。

## 5. Publisher workflow

### 5.1 reserve

`acquire_for_publish()` は round-robin で候補を探索し、各 block に対して次を行う。

1. `refcount == 0` を確認する。
2. timestamp が0、または publish から100 ms以上経過していることを確認する。
3. `refcount` を CAS で `0 -> 1` に変更する。
4. CAS に失敗したら次の候補へ進む。
5. `uid` を1増加する。
6. `publish_timestamp_us` を0へ戻す。
7. reservation を返す。

Publisher reservation が存在する間は、別の Publisher が同じ block を取得できない。
Subscriber は `refcount != 0` だけを理由に拒否せず、uid の前後再確認で Publisher
との競合を検出する。

### 5.2 GPU work と ready event

Publisher は取得した device pointer へ GPU work を enqueue し、同じ依存関係を持つ
CUDA stream で `prepare_publish()` を呼ぶ。この操作は descriptor を作成し、ready event
を記録し、Publisher reservation を commit してから descriptor を返す。commit は次の
処理を行う。

1. reservation の uid が現在の uid と一致することを確認する。
2. `publish_timestamp_us` を現在時刻へ更新する。
3. Publisher reservation の `refcount` を1減少させる。

commit は Publisher reservation を解放する。Subscriber buffer reference が残っていれば block は
再利用できない。`prepare_publish()` 後の middleware publish 結果は block lifecycle へ
反映されない。descriptor が middleware へ渡されなかった場合も通常の再利用条件に従う。

準備処理のいずれかが失敗した場合、`prepare_publish()` は descriptor を返さない。
その block の reservation は解放せず、`GpuBufferManager::reset()` まで再利用しない。
library は自動回復を行わない。

### 5.3 cancel

`prepare_publish()` 前に `PublishBlock` を破棄するか `cancel()` を呼ぶと、Publisher
reservation の `refcount` だけを減少させる。cancel では publish timestamp を更新しないため、
未公開の reservation は grace period を追加で発生させない。準備に失敗した block は既に
再利用禁止になっているため、destructor と `cancel()` は reservation を解放しない。

## 6. Subscriber acquire と release

Subscriber は message の `shm_name`、`publisher_instance_id`、`block_id`、`uid`
を使って buffer reference を取得する。

acquire は次を行う。

1. shared memory へ attach し、layout と block 範囲を検証する。
2. Publisher instance ID を検証する。
3. message の uid と block の uid を確認する。
4. `refcount` を CAS で1増加する。
5. uid を再確認する。
6. 再確認に失敗した場合は refcount を戻して失敗する。
7. 成功したら内部の buffer reference を保持する `ReadHandle` を返す。

`ReadHandle` の破棄時には consumer stream へ completion event を記録する。event、
imported resource 参照、buffer reference は内部 deferred queue へ移され、queue が
項目を順次 `cuEventSynchronize` で待った後に handle 固有の参照が解放される。cache
entry の破棄方針は import cache が管理する。

`BufferMapper::map(message, consumer_stream)` が producer ready event の wait を指定
stream へ enqueue するため、利用者は返された `ReadHandle` から device pointer を取得
してGPU workをenqueueするだけでよい。mapperへ渡したstreamは、対応するhandleの破棄と
completion eventの記録が完了するまで有効でなければならない。

## 7. Publisher/Subscriber 競合の安全性

Publisher が先に refcount を claim した場合、Subscriber が古い uid を使って
refcount を増加できても、uid の再確認に失敗して refcount を戻す。

Subscriber が先に refcount を増加した場合、Publisher の `0 -> 1` CAS が失敗する。

```text
Publisher                              Subscriber

refcount 0 -> 1 を CAS
uid を更新
                                        uid を確認
                                        refcount を増加
                                        uid を再確認
                                        失敗時は refcount を減少
publish_timestamp_us を更新
Publisher ref を解放
```

この手順により、Publisher が block を再利用する一方で古い uid の Subscriber
buffer reference が成立することを防ぐ。

## 8. 失敗時の挙動と既知の制約

| failure | behavior |
| --- | --- |
| reusable block がない | Publisher の acquire が失敗する |
| GPU resource 初期化失敗 | 作成済み resource を rollback する |
| preparation 失敗 | reservation を保持し、manager reset まで再利用しない |
| uid mismatch | Subscriber acquire または reservation 完了を失敗させる |
| Subscriber process crash | refcount が残り、block が再利用不能になる可能性がある |
| 100 ms を超える message 遅延 | block 再利用後は uid mismatch で drop される可能性がある |

固定 grace period は配送保証ではない。publish rate、最大 Subscriber 遅延、block 数に
応じて、遅延 message を drop し得る安全側の設計である。

`uid` は wire format と shared state の `uint32_t` を維持する。wraparound 時に
非常に古い message と一致する可能性があるため、必要なら Publisher instance と shared
memory pool を再生成する。

shared memory 上の atomic は Linux x86_64/aarch64 と対象 toolchain に依存する。現在の
実装は aligned な整数 field を process-shared atomic として扱うため、異なる ABI 間の
shared-memory 相互運用は保証しない。
