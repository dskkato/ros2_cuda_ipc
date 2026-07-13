# Publisher バッファ管理の責務分割方針

## 目的

Publisher 側の `GpuLeasePool` は、GPU allocation、CUDA IPC event、backend 固有の
状態、slot の lease lifecycle、そして publish 用 `BufferView` の組み立てを同時に
扱っている。この方針ではこれらを独立した責務に分け、Publisher helper が slot の
内部状態や backend 実装へ直接依存しない構造へ移行する。

対象は Publisher 側のバッファ確保・再利用・publish 準備である。Subscriber 側の
`BufferViewMapper`、`LeaseHandle` の acquire/release プロトコル、および ROS message
の wire format はこの変更だけでは変更しない。

## 前提と判断

| 項目 | 方針 |
| --- | --- |
| slot の所有者 | `GpuBufferPool` が GPU allocation、ready event、backend 固有 resource を一括所有する。 |
| lifecycle の所有者 | `SlotController` が `LeaseHandle` を用いた空き slot 選択、generation、pending、TTL、cancel を担当する。 |
| publish 中の権限 | `PublishSlot` は acquire 成功後の一回の publish に必要な device pointer、event、descriptor を提供する。 |
| 外部 API | `GpuBufferController` を Publisher helper 向け facade とし、内部 slot を露出しない。 |
| ROS 依存 | resource/lifecycle/descriptor は ROS 非依存にする。ログは core 側で抽象化するか、呼び出し元から注入する。 |
| CUDA 依存 | GPU memory と `cudaEvent_t`/`cudaIpcEventHandle_t` を扱うため CUDA 依存は残る。ここでいう ROS 非依存は CUDA 非依存を意味しない。 |

`BufferDescriptor` は ROS message ではなく、Publisher 側の publish metadata を表す
値型とする。ROS の `BufferCore` への変換は adapter/mapper 境界に閉じ込める。

## 目標構成

```text
ImagePublisherHelper / 他の Publisher helper
                 |
                 v
       GpuBufferController  (facade)
          |              |
          v              v
   GpuBufferPool     SlotController
   resource owner    lifecycle state
          \              /
           \            /
             PublishSlot
          per-publish capability
                 |
                 v
          BufferDescriptor
                 |
                 v
      ROS adapter: BufferCore / BufferView
```

### `GpuBufferPool`: resource ownership

`GpuBufferPool` は初期化済み slot の物理リソースだけを保持する。少なくとも以下を担う。

* 指定 device 上の slot ごとの GPU allocation の確保・破棄
* `cudaEventDisableTiming | cudaEventInterprocess` の ready event の作成・破棄と IPC event handle の取得
* `MemoryBackend` による CUDA IPC / VMM FD 固有 resource の生成・破棄
* pool の構成（slot 数、byte size、device id、backend）と slot index から resource を引く操作

pool は `LeaseHandle`、generation、pending、TTL、Subscriber 数を知らない。resource は
初期化から reset まで不変とし、generation の更新だけで再 allocation や backend resource
の作り直しを行わない。

内部の resource record は公開しない。概念上は次の情報を持つ。

```cpp
struct GpuBufferResource {
  void* device_ptr;
  cudaEvent_t ready_event;
  MemoryBackendKind backend;
  MemoryHandlePayload memory_handle;
  cudaIpcEventHandle_t event_handle;
  // backend 固有の所有状態（VMM allocation、FD server 等）
};
```

### `SlotController`: lifecycle state

`SlotController` は `shm_name` と slot 数を受け、`LeaseHandle` の Publisher 側 API を
呼び出す。責務は以下に限定する。

* `refcnt == 0 && pending == 0` の再利用可能 slot の round-robin 選択
* 選択 slot の generation 更新と publish 時点の pending 設定
* slot ごとの pending deadline の保持と stale pending の回収
* kernel launch、event record、ROS publish の失敗時に pending を取り消す操作

resource の有無を判定したり CUDA API を呼んだりしない。`reserve(subscriber_count)` は
`slot` と `generation` を持つ予約結果を返し、予約の完了または取消しを一度だけ受け付ける。
これにより「generation を進めたが publish できなかった」状態を、呼び出し側の任意の
cleanup ではなく controller の状態遷移として扱える。

### `PublishSlot`: per-publish capability

`PublishSlot` は `GpuBufferPool` の resource と `SlotController` の予約結果を結合した、
短命・move-only の capability とする。公開する情報は次に限る。

* GPU 書き込み先の `device_ptr()`
* 書き込み完了後に record する `ready_event()`
* `slot()` と `generation()`
* `descriptor()` による publish metadata の生成

`PublishSlot` は自ら allocation を破棄せず、slot を再選択しない。失敗時の `cancel()` は
対応する予約の pending を解除する。成功時には `commit()`（または `release()`）で予約を
完了扱いにする。破棄時に未完了の reservation を自動 cancel するかは、既存 API との
互換性を保つため初回実装前に明確化する。

推奨は **未 commit の `PublishSlot` をデストラクタで cancel する** 方針である。これにより
kernel launch、event record、ROS publish のいずれで早期 return しても pending を残しにくい。
ただし publish が成功した後は、Subscriber が pending を消費するまで残す必要があるため、
ROS publish が受理された時点で必ず `commit()` を呼ぶ契約にする。

### `GpuBufferController`: Publisher 用 facade

`GpuBufferController` は `GpuBufferPool` と `SlotController` を合成し、Publisher helper が
使う唯一の core API とする。

* `initialise(byte_size, device_id)` / `reset()` / `matches(...)`
* `reclaim_stale_pending()`
* `acquire_for_publish(subscriber_count) -> optional<PublishSlot>`

facade は resource slot と lifecycle slot が同じ index を参照することを検証して
`PublishSlot` を構築する。Publisher helper は `PublishSlot` から pointer/event/descriptor を
得るだけであり、`GpuBufferPool::Slot` や `LeaseHandle` を参照しない。

## ROS 非依存 descriptor

次の値型を `ros2_cuda_ipc_core` の ROS 依存しないヘッダに置く。`MemoryHandlePayload` は
既存の固定長 payload をそのまま再利用する。

```cpp
struct BufferDescriptor {
  MemoryBackendKind backend;
  MemoryHandlePayload memory_handle;
  cudaIpcEventHandle_t event_handle;

  std::string shm_name;
  uint32_t slot;
  uint32_t generation;

  int device_id;
  uint64_t byte_size;
};
```

`PublishSlot::descriptor()` は `GpuBufferPool` の不変 resource 情報と `SlotController` の
予約 generation をコピーして返す。descriptor は device pointer や `cudaEvent_t` を含めない。
これらは Publisher プロセス内の書き込み用 capability であり、プロセス間に配布する情報ではないためである。

ROS 境界には次の変換を追加する。

```text
BufferDescriptor <-> ros2_cuda_ipc_msgs::msg::BufferCore
BufferDescriptor + local ready_event/device_ptr -> view::BufferView
```

後者は Publisher 内で既存の `ImageView` を作る場合だけに必要である。可能であれば
Publisher helper は `ImageView` を経由せず、`BufferDescriptor` と画像 layout から ROS message
を直接構築する。これにより Publisher が Subscriber 向け RAII view を生成する現在のねじれを解消する。

## Publisher の処理フロー

```text
reclaim stale pending
        |
        v
acquire_for_publish(subscriber_count)
        |  (空きなし: publish を見送る)
        v
PublishSlot { pointer, ready event, slot/generation }
        |
        v
GPU kernel writes pointer -> cudaEventRecord(ready event)
        |  (失敗: PublishSlot::cancel / destructor)
        v
descriptor を ROS message に変換して publish
        |  (失敗: PublishSlot::cancel / destructor)
        v
PublishSlot::commit
```

`subscriber_count == 0` の場合も generation は更新するが pending は 0 とする、という現行挙動を維持する。
TTL 回収は `pending > 0 && refcnt == 0` の slot にだけ適用する。`commit()` は pending を消費・解除しない。
それは Subscriber の初回 acquire、または TTL による回収の責務である。

## 移行手順

1. `BufferDescriptor` と `BufferCore` 変換関数を追加し、変換の単体テストを追加する。ROS message の field 名
   （`slot_id`）と descriptor の field 名（`slot`）の対応をこの層だけに閉じ込める。
2. 現行 `GpuLeasePool::Slot` を resource 専用の非公開 record に置き換え、`GpuBufferPool` と
   `MemoryBackend` の allocation/destroy API を移す。CUDA IPC と VMM FD の両 backend について
   allocation と cleanup の既存テストを維持する。
3. `SlotController` を導入し、`acquire`、`reclaim_stale_pending`、`cancel_pending` の lease 操作を移す。
   generation/pending/TTL の既存テストを controller のテストに移植する。
4. `PublishSlot` を導入し、resource と reservation を結合する。失敗経路、二重 cancel、commit 後の破棄、
   例外または early return 時の pending cleanup をテストする。
5. `GpuBufferController` を導入して facade 経由で上記を組み立てる。既存 `GpuLeasePool` はこの段階で
   廃止するか、短期間の互換 wrapper に縮小する。
6. `ImagePublisherHelper` を facade API に移行する。kernel は `publish_slot.device_ptr()`、event record は
   `publish_slot.ready_event()`、message 作成は `publish_slot.descriptor()` を使う。
7. `GpuLeasePool` と `buffer_view_from()` への参照を削除し、公開ヘッダ、CMake、開発資料、テスト名を更新する。

各段階で public API の追加と呼び出し側の移行を分離し、CUDA IPC と VMM FD の両 backend を同じテスト観点で
確認する。複数段階にまたがって `GpuLeasePool` と新 facade を同じ SHM 名で併用してはならない。

## テスト観点と完了条件

| 観点 | 確認内容 |
| --- | --- |
| resource ownership | initialise/reset の繰り返しで event と backend resource を一度だけ作成・破棄する。 |
| slot lifecycle | 空き slot 選択、generation 更新、pending 設定、TTL 回収が現行と同じ lease 条件を満たす。 |
| failure safety | kernel、event record、ROS publish の各失敗で pending が残らず、commit 後には cancel されない。 |
| descriptor | backend payload、event handle、SHM 名、slot/generation、device id、byte size が `BufferCore` と完全に往復する。 |
| encapsulation | Publisher helper が pool 内部 slot、`LeaseHandle`、backend state に直接アクセスしない。 |
| backend parity | CUDA IPC と VMM FD が同一の facade/`PublishSlot` API で publish metadata を生成できる。 |

完了時には、Publisher helper が lifecycle と resource ownership の詳細を知らずに「slot を取得し、GPU に書き、
event を記録し、descriptor を publish して commit する」だけになることを受け入れ条件とする。
