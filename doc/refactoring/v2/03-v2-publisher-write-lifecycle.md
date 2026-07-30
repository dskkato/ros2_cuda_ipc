# PR 03: V2 publisher write lifecycle

## 目的

publisher側をreservation/lease語彙から、buffer slotのexclusive writer lifecycleへ移す。V2 C++ APIは `GpuBufferManager::try_acquire_for_write()` とmove-only `BufferWrite` を提供する。

## 公開API契約

```cpp
auto write = manager.try_acquire_for_write();
if (!write) { /* reusable slotなし */ }

launch_producer_work(write->device_ptr(), stream);
auto descriptor = write->commit(stream);  // Expected<BufferDescriptor, WriteError>
```

- `try_acquire_for_write()` はIdleかつ `reusable_after` 到達済みのslotだけを取得する。
- 取得成功時、managerは `access_state` をwriter状態へ遷移し、generationを進めてから `BufferWrite` とdevice pointerを返す。これ以降、古いgenerationのreaderは成立しない。
- `BufferWrite::commit(stream)` は同じproducer dependencyを持つstreamへready eventをrecordし、`BufferDescriptor` を構築し、`published_at` と `reusable_after` を更新してwriter referenceを解放する。戻り値はV2 `Expected` とする。
- `commit()` 成功後のROS middleware publish成否はslot lifecycleを変更しない。
- `BufferWrite` はcopy不可・move可能。未commitの通常破棄はcancel扱いにする。

## cancel と異常系

`BufferWrite::cancel(stream)` はproducer completion eventをrecordし、そのevent完了後にwriter referenceをreleaseする。producer workをenqueue済みでも、次のwriterが同じslotを上書きしない。

ready event record、completion event record、metadata更新、resource操作の安全性が証明できない失敗はslotをquarantineする。quarantine slotはmanagerの明示的 `reset()` または再初期化までwriter取得できない。failure時はstable failure codeを含むstructured logを必ず出す。metrics registry、timeout回収、subscriber主導の復帰は実装しない。

## 再利用grace period

`GpuBufferManager::Config` にpublisher設定のreuse grace periodを追加し、defaultを100 msにする。commit時に `reusable_after = published_at + grace_period` をmetadataへ記録する。readerが0でも期限前はwriter取得を拒否し、subscriberがmapする前の遅延messageを安全側にdropできるようにする。

## テストと受け入れ条件

- Idle slotだけをwriter取得でき、writer/reader競合ではone-writer-or-readersを守る。
- writer取得でgenerationが先に進み、古いdescriptorのreader取得が失敗する。
- commitでdescriptor、ready event、published/reusable時刻が整合する。
- cancelはcompletion event前にslotを再利用させず、完了後に再利用可能にする。
- commit/cancelの失敗はquarantineし、reset/reinitializationだけが復帰させる。
- V1 publisher APIとtestはgreenのまま、V2 testを独立追加する。
