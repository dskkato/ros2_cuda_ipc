# Publisher バッファ管理の責務分割方針

## 目的

Publisher 側の `GpuLeasePool` は、現在、次の責務を一つの class に集約している。

- GPU allocation の確保と破棄
- CUDA IPC event の作成と破棄
- memory backend 固有 resource の所有
- slot の lease lifecycle 管理
- publish に使う slot の選択
- generation と pending の更新
- stale pending の回収
- publish 失敗時の rollback
- publish 用 metadata の生成
- Publisher 内部用 `BufferView` の生成

これらは lifetime、更新頻度、依存先が異なる責務である。特に、長寿命な GPU resource の所有と、publish ごとに変化する slot lifecycle が同じ `Slot` に保持されているため、Publisher helper が pool 内部状態や backend 実装へ直接依存しやすい。

本方針では、これらを次の単位に分割する。

- `GpuBufferPool`: 長寿命な GPU resource の所有
- `SlotController`: publish と lease に関する lifecycle state の管理
- `PublishSlot`: 一回の publish に必要な capability
- `GpuBufferController`: Publisher helper 向け facade
- `BufferDescriptor`: ROS 非依存の publish metadata

対象は Publisher 側のバッファ確保、再利用、publish 準備である。

Subscriber 側の `BufferViewMapper`、`LeaseHandle` の acquire/release protocol、および既存 ROS message の wire format は、この refactoring だけでは変更しない。

## 設計上の判断

| 項目 | 方針 |
| --- | --- |
| GPU resource の所有者 | `GpuBufferPool` が GPU allocation、ready event、backend 固有 resource を一括所有する。 |
| slot lifecycle の所有者 | `SlotController` が shared lifetime state、generation、pending、TTL、cancel を管理する。 |
| publish 中の capability | `PublishSlot` が一回の publish に必要な device pointer、ready event、slot/generation、descriptor を提供する。 |
| Publisher helper 向け API | `GpuBufferController` を唯一の facade とし、内部 resource record や `LeaseHandle` を露出しない。 |
| ROS 依存 | resource、lifecycle、reservation、descriptor は ROS 非依存にする。 |
| CUDA 依存 | GPU allocation、`cudaEvent_t`、`cudaIpcEventHandle_t` を扱うため CUDA 依存は残る。 |
| Publisher の View | 目標構成では Publisher は Subscriber 向け `BufferView` / `ImageView` を生成しない。 |
| publish metadata | `BufferDescriptor` を ROS 非依存の値型とし、ROS message への変換は adapter 境界に置く。 |

ここでいう ROS 非依存は CUDA 非依存を意味しない。

memory sharing backend は CUDA IPC と VMM + FD を切り替えられるが、ready event の process 間共有には、どちらの backend でも CUDA IPC event handle を用いる。

## 目標構成

```text
ImagePublisherHelper / other Publisher helpers
                    |
                    v
          GpuBufferController
              facade
             /      \
            v        v
   GpuBufferPool   SlotController
   resource owner  lifecycle owner
            \        /
             \      /
              v    v
             PublishSlot
        per-publish capability
                    |
                    v
           BufferDescriptor
                    |
                    v
      ROS adapter -> BufferCore
```

責務は次のように分かれる。

```text
GpuBufferPool
  resource を所有する

SlotController
  いつ、どの slot を publish に使えるかを決める

PublishSlot
  一回の publish で許可された操作を提供する

GpuBufferController
  resource と lifecycle を組み合わせて PublishSlot を返す

BufferDescriptor
  process 間に公開する metadata を表す
```

## `GpuBufferPool`: GPU resource ownership

`GpuBufferPool` は、初期化から reset まで lifetime が続く GPU resource を所有する。

ここでいう GPU buffer resource には、GPU allocation だけでなく、その slot への書き込み完了を示す ready event も含める。allocation と ready event は slot ごとに一対一であり、同じ初期化・破棄 lifetime を持つため、同じ pool が所有する。

### 責務

- 指定 device 上で slot ごとの GPU allocation を確保する
- slot ごとに ready event を作成する
- ready event の IPC handle を取得する
- CUDA IPC / VMM + FD backend 固有 resource を生成する
- allocation、event、backend resource を破棄する
- pool の構成を保持する
  - slot 数
  - byte size
  - device id
  - memory backend
- slot index から resource を引く

`GpuBufferPool` は次の情報を知らない。

- `LeaseHandle`
- `refcnt`
- `pending`
- `generation`
- pending deadline
- Subscriber 数
- ROS message
- publish 成否

resource は pool の初期化から reset まで不変とする。slot の generation が変わっても、GPU allocation、memory handle、ready event、event handle は再作成しない。

### 内部 resource record

resource record は非公開とする。概念上は次の情報を持つ。

```cpp
struct GpuBufferResource {
  void* device_ptr = nullptr;
  cudaEvent_t ready_event = nullptr;

  MemoryBackendKind backend;
  MemoryHandlePayload memory_handle;
  cudaIpcEventHandle_t event_handle{};

  std::shared_ptr<BackendState> backend_state;
};
```

`BackendState` は VMM allocation や FD server など、backend 固有 resource の所有状態を保持する。

## `SlotController`: publish lifecycle state

`SlotController` は、slot の GPU resource ではなく、publish と lease に関する lifecycle state を管理する。

### 責務

- lease shared state の初期化と終了
- `refcnt == 0 && pending == 0` の再利用可能 slot の選択
- 選択 slot の generation 更新
- publish 時点の pending 設定
- slot ごとの pending deadline の管理
- stale pending の回収
- publish 準備失敗時の pending 取消し
- reservation の commit/cancel 状態管理

CUDA API は呼ばない。GPU allocation や memory handle の存在も知らない。

### reservation

`reserve(expected_consumers)` は、slot と generation を持つ move-only な reservation を返す。

```cpp
class SlotReservation {
 public:
  SlotReservation(SlotReservation&&) noexcept;
  SlotReservation& operator=(SlotReservation&&) noexcept;

  SlotReservation(const SlotReservation&) = delete;
  SlotReservation& operator=(const SlotReservation&) = delete;

  ~SlotReservation();

  uint32_t slot() const noexcept;
  uint32_t generation() const noexcept;

  void commit_publish();
  void cancel();
};
```

`SlotReservation` は次の状態を持つ。

```text
reserved
  |
  +-- commit_publish() --> committed
  |
  +-- cancel() ---------> cancelled
  |
  +-- destructor -------> cancelled
```

`commit_publish()` は shared lifetime state の pending を消費または解除しない。pending は `reserve()` 時点ですでに設定され、その後は Subscriber の lease acquire または TTL 回収によって減少する。

`commit_publish()` は destructor による自動 cancel を無効化する process-local な状態遷移である。つまり、message を publish API へ引き渡したため、この reservation を Publisher 側で取り消さないことを確定する操作であり、Subscriber への配送完了を意味しない。

未 commit の reservation は destructor で自動 cancel する。これにより、kernel launch、event record、message 構築、ROS publish API 呼び出し前後の early return で pending を残しにくくする。

## `PublishSlot`: per-publish capability

`PublishSlot` は、`GpuBufferPool` の resource と `SlotController` の `SlotReservation` を結合した、短命で move-only な capability とする。

```text
GpuBufferResource
       +
SlotReservation
       =
PublishSlot
```

公開する操作は次に限定する。

- GPU 書き込み先の `device_ptr()`
- 書き込み完了後に record する `ready_event()`
- `slot()`
- `generation()`
- `descriptor()`
- `commit_publish()`
- `cancel()`

`PublishSlot` は allocation を破棄せず、slot を再選択せず、resource record を外部へ公開しない。

`PublishSlot` は `GpuBufferController` およびその内部 state より長生きしてはならない。この lifetime を raw pointer の暗黙契約だけにせず、shared state の所有、active reservation の検出、または helper の所有構造によって保証する。

## `GpuBufferController`: Publisher helper 向け facade

`GpuBufferController` は `GpuBufferPool` と `SlotController` を合成し、Publisher helper が利用する唯一の core API とする。

### 責務

- config validation
- CUDA device の選択
- `GpuBufferPool` と `SlotController` の transactional initialisation
- 部分初期化失敗時の rollback
- stale pending の回収
- resource slot と lifecycle slot の index 対応の検証
- `PublishSlot` の構築
- reset 時の安全な破棄

通常の publish 経路では、`acquire_for_publish()` が stale pending の回収を内部で行う。

```text
reclaim stale pending
        |
        v
reserve lifecycle slot
        |
        v
resolve GPU resource
        |
        v
construct PublishSlot
```

`GpuBufferController::initialise()` は resource pool と lifecycle state の両方が成功した場合だけ initialised 状態になる。後段の初期化に失敗した場合は、前段で作成した resource をすべて rollback する。

## `BufferDescriptor`: ROS 非依存 publish metadata

`BufferDescriptor` は ROS message ではなく、Publisher が process 間に公開する buffer metadata を表す値型とする。

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

`BufferDescriptor` は device pointer、`cudaEvent_t`、`LeaseHandle`、Publisher 内部 resource pointer、ROS header、画像・点群固有 metadata を含めない。

`PublishSlot::descriptor()` は resource の不変情報と reservation の generation を snapshot としてコピーして返す。一度生成した descriptor は、その後 `PublishSlot` が commit または cancel されても値が変化しない。

## ROS 境界

初回実装で必要な変換は Publisher 向けの一方向だけとする。

```text
BufferDescriptor
      |
      v
ros2_cuda_ipc_msgs::msg::BufferCore
```

ROS message field の `slot_id` と descriptor の `slot` の対応は、この変換層だけに閉じ込める。

逆方向の `BufferCore -> BufferDescriptor` は、Subscriber 側の責務分離に実際に必要であることを確認してから追加する。

目標構成では Publisher helper は `BufferView` や `ImageView` を生成しない。

```text
PublishSlot
    |
    v
BufferDescriptor
    |
    v
BufferCore
    |
    + image metadata
    v
GpuImage
```

既存 helper を段階移行するために一時的な `BufferView` 生成経路が必要な場合は compatibility layer として限定し、最終構成には残さない。

## Publisher の処理フロー

```text
acquire_for_publish(expected_consumers)
        |
        | empty: skip publish
        v
PublishSlot
  - device pointer
  - ready event
  - slot
  - generation
        |
        v
enqueue GPU write
        |
        | failure: automatic cancel
        v
cudaEventRecord(ready_event)
        |
        | failure: automatic cancel
        v
create BufferDescriptor
        |
        v
convert descriptor to ROS message
        |
        | failure: automatic cancel
        v
call ROS publish API
        |
        | exception/local failure: automatic cancel
        v
commit_publish()
```

`commit_publish()` は ROS middleware による配送完了を表さない。ROS publish API の呼び出しが例外なく完了し、Publisher 側で reservation を取り消さないと判断した時点で呼ぶ。

## `subscriber_count == 0` の扱い

現行挙動を維持する。

- generation は更新する
- pending は 0 とする
- publish 後も slot は即時に再利用可能である

後から古い message を受信した Subscriber は、generation 不一致により lease acquire に失敗する。

## stale pending の回収

TTL 回収は、次の条件を満たす slot にだけ適用する。

```text
pending > 0
refcnt == 0
deadline expired
```

`refcnt > 0` の slot は回収しない。TTL 回収は利用中の Subscriber から slot を取り上げる操作ではなく、lease acquire に至らなかった stale pending を回収する操作である。

## 移行手順

1. 現行 `GpuLeasePool::Slot` を resource state と lifecycle state に分ける。
2. `BufferDescriptor` と `BufferDescriptor -> BufferCore` 変換を追加する。
3. lease shared state、generation、pending、TTL、cancel を `SlotController` へ抽出する。
4. allocation、event、backend state を `GpuBufferPool` へ抽出する。
5. move-only な `SlotReservation` を導入し、destructor auto-cancel と `commit_publish()` を実装する。
6. `PublishSlot` と `GpuBufferController` を導入する。
7. `ImagePublisherHelper` を facade API へ移行する。
8. `GpuLeasePool`、`buffer_view_from()`、Publisher 側の `BufferView` 生成経路を削除する。

複数段階にまたがって、旧 `GpuLeasePool` と新 `GpuBufferController` を同じ SHM 名で同時に利用してはならない。

## テスト観点

| 観点 | 確認内容 |
| --- | --- |
| resource ownership | initialise/reset の繰り返しで allocation、event、backend resource を一度だけ作成・破棄する。 |
| transactional initialise | resource または lifecycle の途中失敗で、作成済み state がすべて rollback される。 |
| slot lifecycle | free slot 選択、generation 更新、pending 設定、TTL 回収が現行条件を維持する。 |
| reservation transition | reserved→committed、reserved→cancelled、destructor cancel が正しく動作する。 |
| invalid transition | 二重 commit、二重 cancel、commit 後 cancel、cancel 後 commit を検出する。 |
| failure safety | kernel、event record、message 構築、ROS publish の各失敗で pending が残らない。 |
| commit behavior | commit 後の destructor が pending を cancel しない。 |
| descriptor | backend payload、event handle、SHM 名、slot/generation、device id、byte size を正しく生成する。 |
| descriptor snapshot | descriptor 生成後に reservation state が変わっても値が変化しない。 |
| encapsulation | helper が pool 内部 resource、`LeaseHandle`、backend state に直接アクセスしない。 |
| lifetime safety | `PublishSlot` が controller/resource より長生きする不正利用を検出または防止する。 |
| backend parity | CUDA IPC と VMM + FD が同一 facade API で descriptor を生成できる。 |
| reinitialise | byte size、device、backend、slot count、SHM name の変更時に正しく再構築する。 |
| zero subscriber | pending=0、generation 更新、即時再利用、古い message の generation mismatch を確認する。 |

## 完了条件

refactoring 完了時、Publisher helper は次だけを行う。

```text
PublishSlot を取得する
GPU data を書き込む
ready event を record する
descriptor から ROS message を作る
publish API へ渡す
commit_publish() する
```

Publisher helper は次を知らない。

- slot resource の内部 record
- memory backend 実装
- shared lifetime state の layout
- `LeaseHandle`
- generation の更新方法
- pending の設定・回収方法
- publish 失敗時の pending rollback 手順
- Subscriber 向け `BufferView` の構築方法

最終的な責務分担は次とする。

```text
GpuBufferPool
  GPU resource を所有する

SlotController
  publish と lease の lifecycle を管理する

SlotReservation
  一回の reservation の状態遷移を管理する

PublishSlot
  一回の publish に必要な capability を提供する

GpuBufferController
  resource と lifecycle を統合する

BufferDescriptor
  process 間に公開する metadata を表す

ROS adapter
  BufferDescriptor を BufferCore へ変換する
```
