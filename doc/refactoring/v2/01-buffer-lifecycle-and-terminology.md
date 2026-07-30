# PR 01: V2 buffer lifecycle と用語の正本化

## 目的

V2の設計上の主語を、publisherが所有する共有 GPU buffer slot に置き直す。`lease` は shared metadata 上のreader referenceを実現する低レベル機構であり、公開設計および主要な型名の主語には使わない。このPRは文書だけを追加・更新し、既存実装・wire contract・examplesを変更しない。

## 設計契約

`doc/buffer_lifecycle.md` を新しい正本として追加し、以下を定義する。

```text
Shared GPU Buffer Slot
  ├─ publisher-owned allocation
  ├─ producer ready event
  └─ process-shared BufferSlotMetadata
       ├─ generation
       ├─ access_state: -1 writer / 0 no active access / >0 readers
       ├─ published_at
       ├─ reusable_after
       └─ quarantine state and failure code
```

- publisherはwriter、subscriberはreaderとして同じslot lifecycleへ参加する。
- `access_state == 0` は `reusable_after` 前ならPublished、期限到達後ならIdleと導出する。
- writer取得時、GPU pointerをcallerへ返す前にgenerationを進め、旧publicationを無効化する。
- readerは現generationでのみ取得できる。readerが存在する間はwriterを取得できない。
- V2の通常系はcrash recoveryを含まない。publisher reset/reinitialization、protocol/layout version mismatch、publisher instance mismatchだけを安全側に扱う。
- CUDA操作の安全性を証明できない失敗はslotをquarantineし、publisher managerの明示的 `reset()` または再初期化でだけ復帰させる。

所有権は次のmatrixを正本にする。

| 対象 | 所有者 | active accessとの関係 |
| --- | --- | --- |
| allocation / producer ready event | publisher buffer pool | slotの物理resource |
| metadata mapping | `BufferMapper` | cacheでありreader referenceではない |
| imported CUDA resource | `BufferMapper` | cacheでありreader referenceではない |
| active reader reference | unbound/bound read state | slot再利用を止める |
| GPU完了待ちのreader/resource | process-lifetime deferred queue | completion event後にrelease |
| DLPack export後のread state | DLPack capsule context | framework解放まで保持 |

## 文書変更

- `doc/design.md` はV2を将来の正本として参照するが、V1 APIの説明は削除しない。
- `doc/lease_protocol.md` はV1 current implementation / history候補として注記する。
- `doc/python-subscriber-implementation.md` はV2 Python DLPack設計へtask 05から参照する。
- 本directoryのtask docsを `doc/refactoring-migration.md` からindexする。

## 受け入れ条件

- 新文書だけでwriter/readers、generation、grace period、quarantine、deferred release、cache lifetimeの分離を説明できる。
- 既存文書のV1契約を「V2で維持される」と誤記しない。
- 既存packageのbuildとtestは変更なしでgreenである。

## 前提・範囲外

- V2はV1の互換renameではなく、parallel V2 contractである。
- crash liveness、heartbeat、timeout回収、subscriber crashからのreader強制回収は後続課題。
