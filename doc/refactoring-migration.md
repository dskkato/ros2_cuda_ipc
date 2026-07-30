# Buffer-centric lifetime modelへの移行アウトライン

## V2 実行タスク design docs

このアウトラインは背景と移行上の検討事項を残す。合意済みのV2実装は、以下の文書をそれぞれ単独でbuild・test可能なPRとして実施する。V1との互換は同一workspace内の並行提供であり、旧新をbridgeしない。

1. [01-buffer-lifecycle-and-terminology.md](refactoring/v2/01-buffer-lifecycle-and-terminology.md)
2. [02-v2-wire-schema-and-coexistence.md](refactoring/v2/02-v2-wire-schema-and-coexistence.md)
3. [03-v2-publisher-write-lifecycle.md](refactoring/v2/03-v2-publisher-write-lifecycle.md)
4. [04-v2-subscriber-read-lifecycle.md](refactoring/v2/04-v2-subscriber-read-lifecycle.md)
5. [05-v2-python-dlpack.md](refactoring/v2/05-v2-python-dlpack.md)
6. [06-v2-examples-and-documentation-cutover.md](refactoring/v2/06-v2-examples-and-documentation-cutover.md)

V2の正本は task 01 が追加する `doc/buffer_lifecycle.md` とする。各taskは、この一覧と前提taskに記載した公開契約を満たすことを受け入れ条件とする。

## 1. 背景

現在の実装は、publisherが所有するGPU buffer slotを複数subscriberから安全に利用するため、shared memory上のgenerationとrefcountを用いている。

現行設計では、この仕組みを「lease protocol」として説明し、次の概念を中心に構成している。

* Publisher reservation
* Subscriber lease
* `LeaseManager`
* `LeaseHandle`
* `LeaseMapping`
* publication lease
* `MappedPublication`
* `ReadHandle`

このモデルは現在の安全性を説明できている一方、実装上は比較的単純なslot利用状態まで独立したprotocolとして強調している。

その結果、次のような問題がある。

* bufferよりleaseが中心概念に見える
* refcount操作が独立したdomain modelに見える
* publisherとsubscriberが同じbuffer lifecycleへ参加していることが見えにくい
* resource import、generation、refcount、stream synchronizationの関係が複数の型へ分散している
* documentationの構造が将来の実装整理を制約するアンカーになっている

今回の目標は、既存の公開APIと安全性を維持しながら、設計の中心をleaseからbufferへ移すことである。

---

## 2. 目標とするmental model

### 2.1 中心概念

中心となるのは、publisherが所有する共有GPU buffer slotである。

```text
Shared GPU Buffer Slot
  ├─ GPU allocation
  ├─ producer ready event
  └─ shared metadata
       ├─ generation
       ├─ access state / refcount
       └─ publish timestamp
```

publisherとsubscriberは別々のprotocolを実行する主体ではなく、同じbuffer lifecycleへ異なる立場で参加する。

```text
Publisher
  idle bufferをwrite用に取得
  GPU workを実行
  ready eventをrecord
  bufferをpublish可能状態にする

Subscriber
  published bufferへのread参照を取得
  ready eventをconsumer streamで待つ
  GPU workを実行
  完了後にread参照を解放する
```

### 2.2 leaseの位置づけ

leaseは独立した中心概念ではなく、bufferが利用中であることを表す内部的な参照管理である。

```text
subscriber lease
  ≒ buffer metadata上のreader reference

publisher reservation
  ≒ buffer metadata上のexclusive writer reference
```

`lease`という語を完全に禁止する必要はないが、設計説明、型名、ディレクトリ構成の中心には置かない。

### 2.3 one-writer-or-readers

buffer slotは、概念的には次のいずれかの状態にある。

```text
Idle
Writing
Published / readable
Reading by N readers
```

排他制御はgenerationとatomic access stateで表現する。

単純な候補は次である。

```cpp
// 0: idle
// -1: writer acquired
// >0: active reader count
std::atomic<int32_t> access_state;
```

実際の状態表現は既存の`refcnt`方式との互換性、generation競合、grace period、異常系を考慮して決定する。

---

## 3. 維持する公開API

今回マージしたsubscriber APIは、buffer-centric modelとも整合しているため維持する。

### C++

```cpp
auto read = mapper.map(message, consumer_stream);
```

`ReadHandle`は、一つのstream-bound GPU readを表す。

### DLPack

```text
map_for_dlpack(message)
  -> stream-unbound view

__dlpack__(stream)
  -> stream-bound read state
  -> DLPack capsule
```

DLPackではconsumer streamが遅れて決まるため、stream-unbound状態が必要になる。

この中間状態は公開APIへ露出させず、内部実装として保持する。

---

## 4. 現在のモデルからの対応関係

| 現在の概念                  | 移行後の位置づけ                               |
| ---------------------- | -------------------------------------- |
| GPU buffer slot        | 中心概念として維持                              |
| `SlotMeta`             | `BufferMetadata`相当へ再定義                 |
| `refcnt`               | buffer access stateの一部                 |
| Publisher reservation  | exclusive writer acquisition           |
| Subscriber lease       | reader reference                       |
| `LeaseManager`         | buffer pool / metadata管理へ吸収            |
| `LeaseHandle`          | metadata referenceまたはaccess tokenへ縮小   |
| `LeaseMapping`         | buffer metadata mappingへ改名             |
| `MappedPublication`    | stream-unbound subscriber buffer state |
| `ReadHandle`           | stream-bound read operation            |
| deferred release queue | GPU完了後にreader referenceを解放する仕組み        |

この段階では、最終的な型名を先に固定しない。

先に不変条件と状態遷移を決め、その後に名前を整理する。

---

## 5. 設計上の不変条件

移行中も次の安全性を維持する。

### 5.1 Writer exclusion

writerがbuffer slotを取得している間、新しいreaderは成立しない。

### 5.2 Reader protection

readerが一つでも存在する間、publisherは同じslotをwrite用に再取得できない。

### 5.3 Generation validation

古いmessageからのreader取得は、generation確認によって拒否される。

必要であれば、reader count更新前後でgenerationを確認する。

### 5.4 GPU completion

`ReadHandle`のC++ lifetime終了は、GPU read完了を意味しない。

consumer streamへcompletion eventをrecordし、そのevent完了後にreader referenceを解放する。

### 5.5 Resource mapping independence

subscriber process内のimported CUDA resource cacheのlifetimeと、buffer slotのreader referenceは別である。

cache entryが残っていても、publisherのslot再利用を止めない。

### 5.6 Failure conservatism

CUDA event recordや同期処理に失敗した場合、bufferを早期に再利用可能にしない。

安全性が証明できない状態はquarantineまたは再利用禁止とする。

---

## 6. Documentation migration

documentationは、実装変更より先行しすぎず、ただし旧mental modelを固定し続けないよう段階的に移行する。

### Phase A: 新しい設計意図を追加

新しい文書を追加する。

候補:

```text
doc/buffer_lifecycle.md
```

この文書では、次のみを定義する。

* buffer slotの所有者
* buffer metadata
* writer/readersの状態遷移
* generation
* producer/consumer CUDA synchronization
* deferred reader release
* crash時の既知の制約

`lease`はrefcount実装を説明する補助語としてのみ使用する。

既存の`lease_protocol.md`には冒頭に注記を追加する。

```text
This document describes the current implementation terminology.
The target mental model is defined in buffer_lifecycle.md.
```

この時点では既存文書を削除しない。

### Phase B: 正本を切り替える

`design.md`からの参照先を、

```text
lease_protocol.md
```

から、

```text
buffer_lifecycle.md
```

へ切り替える。

`lease_protocol.md`は次のいずれかに縮小する。

* 旧実装の詳細
* migration note
* history document

この段階で、設計上の正本はbuffer lifecycle文書になる。

### Phase C: 旧文書をhistory化

実装と命名の移行完了後、

```text
doc/lease_protocol.md
```

を削除するか、

```text
doc/history/lease_protocol.md
```

へ移動する。

旧文書へのリンク、用語、図を全体検索して除去する。

---

## 7. 実装移行の段階

### PR 1: Buffer-centric design document

目的:

* 新しいmental modelを明文化する
* 既存実装の動作は変更しない
* lease文書を即座に削除しない

変更内容:

* `doc/buffer_lifecycle.md`を追加
* `doc/design.md`へtarget modelを追記
* `lease_protocol.md`をcurrent implementation descriptionとして位置づける
* terminology mappingを追加
* 不変条件と状態遷移をテスト可能な形で記述

このPRでは型名やディレクトリ名を変更しない。

### PR 2: Metadata modelの整理

目的:

* shared memory上の状態をlease固有ではなくbuffer metadataとして扱う
* wire formatと動作は維持する

候補変更:

```text
SlotMeta
  -> BufferSlotMetadata

LeaseMapping
  -> BufferMetadataMapping

LeaseMappingCache
  -> BufferMetadataCache
```

`refcnt`はそのままでもよい。

重要なのは、名前の変更だけではなく、

```text
shared metadata belongs to a buffer slot
```

という依存方向へ変えること。

互換性が必要なら、旧型名を内部aliasとして一時的に残す。

### PR 3: Publisher側のlease概念縮小

目的:

* `LeaseManager`をpublisher reservation managerではなくbuffer slot state managerとして再構成する

候補構成:

```text
GpuBufferManager
  ├─ GpuBufferPool
  └─ BufferStateManager
```

または、規模が小さければ`GpuBufferManager`へ直接吸収する。

処理は次のbuffer operationとして表す。

```text
try_acquire_for_write()
commit_write()
cancel_write()
```

`reservation`という語は、move-onlyなpublish transactionを表す場合には残してよい。

ただし、その内部実装としてlease protocolを意識させない。

### PR 4: Subscriber側のlease概念縮小

目的:

* subscriberのreader reference取得をbuffer mapping処理の一部へする
* `LeaseHandle`を中心概念から外す

候補構成:

```text
BufferMetadataReference
  └─ acquire_reader()
       -> ReaderReference
```

または、さらに内部化して、

```text
StreamUnboundReadState
  imported resource
  metadata mapping
  active reader reference
```

とする。

`MappedPublication`については、DLPackのために必要なstream-unbound状態として再定義する。

型名は実装責務が固まった段階で決める。

候補:

* `UnboundReadState`
* `PendingRead`
* `ImportedReadState`
* `BufferReadState`

### PR 5: ReadHandle内部の整理

目的:

* `ReadHandle::Impl`に集まっている責務を名前のある内部型へ分ける
* public APIは変更しない

候補構成:

```text
ReadHandle
  └─ BoundReadState
       ├─ ImportedResourceReference
       ├─ ReaderReference
       ├─ consumer stream
       └─ completion event
```

破棄時には、

```text
BoundReadState
  -> DeferredReadRelease
```

へmoveする。

deferred queueはprocess lifetimeの内部serviceとして維持する。

### PR 6: atomic state modelの再評価

目的:

* 現在の`refcnt`モデルを維持するか
* one-writer-or-readersを明示したatomic stateへ変えるか
* generationとstateを別atomicにするか、一体化するか

を実装・テストに基づいて判断する。

検討対象:

```text
Option A:
  generation
  refcnt

Option B:
  generation
  access_state (-1 / 0 / readers)

Option C:
  packed generation + writer bit + reader count
```

このPRは名称整理とは分離する。

現在の`refcnt`とCASで不変条件を十分に表せるなら、無理にstate representationを変更しない選択も可能である。

---

## 8. テスト移行

テストもlease object単位からbuffer state transition単位へ重心を移す。

### Publisher

* idle slotだけをwriterが取得できる
* readerが存在するとwriter取得に失敗する
* writer commit後にgenerationとtimestampが正しい
* writer cancel後にslotが再取得可能になる
* preparation失敗時にslotが安全側へ残る

### Subscriber

* current generationのreader取得に成功する
* stale generationを拒否する
* writer取得とのraceで不正なreaderが成立しない
* reader取得後はwriter取得に失敗する
* completion event前にはreader countを減らさない
* completion event後にreader countを減らす

### DLPack

* `map_for_dlpack()`時点のbuffer保持条件
* `__dlpack__(stream)`でのreader取得またはstream bind
* 一度だけexportできる
* capsule破棄後、GPU完了を待ってreader referenceが解放される
* bind失敗時にbufferが早期再利用されない

### 異常系

* subscriber process crash
* publisher crash
* event record失敗
* event synchronize失敗
* stale SHM
* publisher instance mismatch
* generation wraparound

異常系の回復機構は、通常のreader/writer modelとは別の章・別の実装課題として扱う。

---

## 9. 移行時の禁止事項

移行中は次を避ける。

* documentationだけ先に全面的に書き換え、実装との対応を失う
* lease関連の型を一括renameするだけで、依存方向を変えない
* public APIへ新しい中間概念を露出する
* DLPackのstream-unbound状態を通常の`ReadHandle`へ無理に統合する
* atomic state representation変更と型構造変更を同じPRで行う
* crash recoveryを通常系のrefcount modelへ混ぜる
* cache lifetimeとbuffer usage lifetimeを統合する

---

## 10. 最終状態

最終的な設計説明は次の形を目指す。

```text
Publisher owns a pool of GPU buffers.

Each buffer has process-shared metadata describing:
  - its generation
  - whether it is being written
  - how many readers are still using it

A subscriber maps the published buffer into its process and creates
a stream-bound ReadHandle.

ReadHandle keeps the buffer readable until the consumer GPU work
has completed.

The shared reference count is an implementation mechanism used to
prevent the publisher from reusing a buffer too early.
```

コード上では、概ね次の構造を目指す。

```text
GpuBufferManager
  ├─ GpuBufferPool
  └─ BufferMetadataManager

BufferMapper
  ├─ ImportedResourceCache
  └─ BufferMetadataCache

stream-unbound buffer read state
  └─ bind(stream)

ReadHandle
  └─ deferred completion release
```

`lease`は必要に応じて内部コメントや低レベル型に残ってもよいが、公開設計の主語にはしない。
