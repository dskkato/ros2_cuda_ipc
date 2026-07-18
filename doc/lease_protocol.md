# GPU Buffer Lease Protocol

## 1. 目的と適用範囲

この文書は、`ros2_cuda_ipc`がGPU buffer slotの再利用をPublisherとSubscriberの
複数process間で調整するために使用するlease protocolの正式仕様である。

このprotocolの目的は、古いmessageや処理中のSubscriberが参照するGPU bufferを
Publisherが安全でないタイミングで再利用することを防ぐことである。対象には次を含む。

- Publisherによるslotの予約とgeneration更新
- publish後、Subscriberがleaseを取得するまでの保護
- Subscriberによるleaseの取得と解放
- 未完了publishのcancel
- stale pendingのTTL回収
- Publisher/Subscriber間の競合制御

ROS middlewareによる配送保証、Subscriber processの死活監視、Publisher再起動時の
状態引き継ぎは、このprotocolの保証範囲外である。

---

## 2. 用語

| 用語 | 定義 |
| --- | --- |
| slot | Publisherが再利用する1個のGPU bufferと、そのready eventおよびlease状態の組 |
| generation | slotがpublish用に予約されるたびに増加する世代番号 |
| reservation | Publisherが1回のpublish試行のために取得した`slot_id`と`generation` |
| pending | messageを受け取ってleaseを取得すると見込まれるSubscriber数 |
| refcnt | 現在そのslotのleaseを保持しているSubscriber数 |
| reserved | Publisherのslot選択・世代更新とSubscriber acquireを排他する一時的なflag |
| lease | Subscriberが対象generationのslotを読み取り中であることを表す所有権 |
| TTL | staleなpendingをPublisherが回収可能になるまでの期間 |

`pending_count`は通常、publish時点のROS subscription countから得る。これは配送数の
保証値ではなく、期待値である。

---

## 3. 構成要素と責務

```text
Publisher process
  GpuBufferManager
    GpuBufferPool       GPU allocationとready eventを所有
    LeaseManager        reservation、pending、generation、TTLを管理
      LeaseHandle       process-shared lease stateを操作
    PublishSlot         1回のpublish試行を表すmove-only object

Subscriber process
  BufferViewMapper
    LeaseHandle         leaseを取得
  BufferView
    LeaseHandle         viewの生存中leaseを保持
```

ROS messageには少なくとも次が含まれる。

- shared memory名
- `slot_id`
- `generation`
- device IDとbyte size
- memory backend種別とmemory handle
- ready event handle

application固有の画像shape、encoding、point cloud layoutなどはlease protocolに含めない。

---

## 4. Process-shared slot state

各slotはPOSIX shared memory上に次の`uint32_t` stateを持つ。

| field | 意味 | 再利用条件への影響 |
| --- | --- | --- |
| `generation` | 現在の世代 | messageの世代一致確認に使用 |
| `refcnt` | activeなSubscriber lease数 | 0でなければ再利用不可 |
| `pending` | lease取得がまだ期待される数 | 0でなければ再利用不可 |
| `reserved` | Publisher更新中の排他flag | 0でなければPublisher reserveとSubscriber acquireを拒否 |

現在のshared memory layout versionは2である。attach時にmagicとlayout versionを検証し、
各APIは`slot_id`がheaderのcapacity範囲内であることを検証する。

slotを再利用できる基本条件は次である。

```text
reserved == 0 && refcnt == 0 && pending == 0
```

この条件の単純な読み取りだけでは競合を防げないため、Publisherは`reserved`をCASで
取得してから`refcnt`と`pending`を再確認する。

---

## 5. Publisher workflow

Publisherの公開APIは次の順序で使用する。

```cpp
auto slot = manager.acquire_for_publish(pending_count);

launch_gpu_work(slot->device_ptr(), stream);
slot->record_ready(stream);

auto descriptor = slot->descriptor();
publisher->publish(make_message(*descriptor));

slot->commit_publish();
```

状態遷移は次のとおりである。

```text
reserved
  ├─ record_ready成功 ──────────────> ready_recorded
  └─ cancel / destructor ───────────> cancelled

ready_recorded
  ├─ descriptor ────────────────────> ready_recorded
  ├─ commit_publish ────────────────> committed
  └─ cancel / destructor ───────────> cancelled
```

### 5.1 reserve

`acquire_for_publish()`は最初にTTLを超えたpendingの回収を試み、その後slotを
round-robinで探索する。各候補slotについて次を行う。

1. `reserved`を`0 -> 1`へCASする。
2. `refcnt == 0 && pending == 0`を再確認する。
3. `generation`を1増加する。
4. `pending`を`pending_count`で初期化する。
5. `reserved`を0へ戻す。
6. `slot_id`と新しい`generation`をreservationとして返す。

候補がなければacquireは失敗する。bufferを強制再利用してはならない。

### 5.2 GPU workとready event

Publisherは取得したdevice pointerへ必要なGPU workをenqueueした後、同じ依存関係を持つ
CUDA streamで`record_ready()`を呼ぶ。ready eventはlibraryがslotごとに所有する。

`descriptor()`は`record_ready()`成功前には失敗する。これにより、未記録のready eventを
含むmessageの生成を防ぐ。

libraryは、すべてのGPU書き込みが`record_ready()`より前にenqueueされたことまでは検証
できない。これはcallerの契約である。

### 5.3 publishとcommit

`commit_publish()`は、ROS publish APIがmessageをmiddlewareへ引き渡した後に呼ぶ。
commitは`PublishSlot`のprocess-localな状態を`committed`へ変更し、destructorによる
cancelを無効にするだけの非失敗操作である。同じslotへの繰り返しcommitはno-opになる。
descriptor取得成功後はstateが`ready_recorded`であるため、正常なpublish workflowでは
外部resourceや競合に起因するcommit失敗は存在しない。

commitは次を意味しない。

- GPU workの完了
- Subscriberへの配送完了
- Subscriberによるlease取得
- slotが再利用可能になったこと

### 5.4 cancel

未commitの`PublishSlot`を破棄するとreservationを自動的にcancelする。明示的な
`cancel()`も同じ操作を行い、複数回呼んでも安全である。

cancelは`reserved`をCASで取得する。短いPublisher/TTL回収との競合ではbounded retryし、
reservationのgenerationが現在も一致し、かつ`refcnt == 0`の場合だけpendingを0へ戻す。
古いreservationから新しいgenerationのpendingを消去してはならない。retry上限、
generation不一致、active leaseによりcancelできない場合はERRORとして記録する。

`GpuBufferManager::reset()`後でも、manager objectが生存している間は
`PublishSlot`のdestructorがshared-memory reservationをcancelできる。

---

## 6. Subscriber acquireとrelease

Subscriberはmessageの`shm_name`、`slot_id`、`generation`を使ってleaseを取得する。

acquireは次を行う。

1. shared memoryへattachし、slot範囲を検証する。
2. `reserved == 0`を確認する。
3. `generation`がmessageと一致することを確認する。
4. `refcnt`をCASで1増加する。
5. `reserved == 0`と`generation`一致を再確認する。
6. 再確認に失敗した場合は`refcnt`を戻し、acquireを失敗させる。
7. `pending > 0`ならCASで1減少する。
8. 有効な`LeaseHandle`を返す。

有効な`LeaseHandle`の破棄またはmove assignmentによるreleaseは`refcnt`を1減少する。
`BufferView`は内部で`LeaseHandle`を保持するため、viewの生存中はslotを再利用できない。

Subscriberはimportしたready eventを自身のCUDA streamで待ってからbufferを読み取る。

---

## 7. Publisher/Subscriber競合の安全性

PublisherのreserveとSubscriber acquireは、次の再確認によって競合を閉じる。

```text
Publisher                              Subscriber

reservedを0 -> 1へCAS
                                        reserved == 0でなければ失敗
refcntとpendingを再確認
generationとpendingを更新
reservedを0へrelease
                                        generationを確認
                                        refcntを増加
                                        reservedとgenerationを再確認
```

Subscriberが先に`refcnt`を増やした場合、Publisherは`refcnt != 0`を観測してそのslotを
選ばない。Publisherが先にgenerationを更新した場合、古いmessageのSubscriberは
generation再確認に失敗して`refcnt`を戻す。

したがって、有効なSubscriber leaseと同じslotの再利用が同時に成立してはならない。

---

## 8. pendingとTTL

`pending`は、message publish後からSubscriberがleaseを取得するまでの区間を保護する。
Subscriber acquireが成功するたび、0より大きいpendingを1減少する。

message drop、subscription countの過大見積もり、Subscriber側の処理中止などにより
pendingが残る場合がある。Publisherはslotごとのprocess-local deadlineを使って、TTLを
超えたpendingを回収する。

TTL回収は次の条件でのみpendingを0にする。

```text
deadline reached && reservedを取得できる && refcnt == 0
```

TTL回収はbackground timerではない。次の場合に実行される。

- `GpuBufferManager::acquire_for_publish()`の先頭
- `reclaim_stale_pending()`の明示呼び出し

TTLを過ぎても、slotがまだ再利用されずgenerationが一致していれば、遅延Subscriberの
acquireが成功する場合がある。slotが再利用された後はgeneration mismatchにより安全に
失敗する。

---

## 9. Protocolが保証すること

前提とAPI契約が守られる限り、このprotocolは次を保証する。

- activeなSubscriber leaseがあるslotをPublisherが再利用しない。
- pendingが残るslotをTTL回収前に再利用しない。
- Publisher reserveとSubscriber acquireの競合で、両者が同じ旧generationを有効と判断しない。
- 再利用後に届いた古いmessageをgeneration mismatchで拒否する。
- cancelが新しいgenerationのpendingを誤って消去しない。
- ready event記録前にPublisherがdescriptorを取得できない。
- 未commitのpublish試行を通常のscope exitでcancelする。

安全性の基本方針は、条件を確認できない場合に誤ったGPU dataを読ませるのではなく、
acquireまたはpublish試行を失敗させることである。

---

## 10. Protocolが保証しないこと

このprotocolは次を保証しない。

- ROS messageがすべてのSubscriberへ配送されること
- すべてのSubscriberがleaseを取得できること
- `pending_count`が実際の配送数と一致すること
- TTL後に到着したmessageを処理できること
- ROS publish API成功後のmiddleware障害を検出すること
- applicationがready eventを正しいstream順序で記録すること
- applicationがready event待機後にbufferを読むこと
- process crash後の自動resource回復

配送数の見積もり不足、TTL超過、generation mismatchなどではmessageをdropし得る。
これは誤ったbufferを読むより安全なfailure modeとして扱う。

---

## 11. Lifetime contract

`PublishSlot`は所有元`GpuBufferManager`への非所有pointerを保持する。すべての
`PublishSlot`は、そのmanager objectより先に破棄しなければならない。

```text
required:
  PublishSlot destruction
    before
  GpuBufferManager destruction
```

この順序に反してmanagerを先に破棄することはAPI contract violationであり、libraryは
shared ownershipによって補償しない。`reset()`はresource操作を無効にするが、manager
objectが生存していれば未commit slotのshared-memory cancelは可能である。

---

## 12. 既知の制約

### 12.1 Subscriber crashによるrefcnt leak

`refcnt`は`LeaseHandle`の正常なreleaseでのみ減少する。Subscriberが`SIGKILL`、process
crash、machine failureなどでdestructorを通らず終了するとrefcntが残り、そのslotは
再利用不能になる。

これはmemory safety上は安全側だが、pool capacityを失うavailability上の問題である。
現在はprocess identity、heartbeat、process death detectionを実装していない。

### 12.2 Publisher restartと同一SHM名

`LeaseHandle::init()`は指定されたshared memory layoutを再初期化する。同じ`shm_name`で
Publisherを再起動した場合、古いSubscriber mapping、古いmessage、古いlease、古いGPU
handleと新しい状態を安全に引き継ぐprotocolにはなっていない。

Publisher instanceごとに一意なSHM名を使用することを推奨する。

```text
/base_name/<publisher_instance_uuid>
```

実装はlocal configurationを超える`slot_id`を検出した場合、取得したreservationを
generation付きでrollbackしてERRORを記録する。ただし、これはcapacity不一致時の
defensive cleanupであり、同じSHM名を複数Publisherが再初期化することを安全にする
仕組みではない。

### 12.3 generation wraparound

wire formatとshared stateのgenerationは`uint32_t`である。長時間稼働してwraparoundすると、
非常に古いmessageと現在のslotが同じgenerationになる可能性がある。

必要に応じてwraparound前にpoolとSHM名を再生成する。`uint64_t`化はwire format変更を伴う。

### 12.4 Process-shared atomicの可搬性

現在の実装はshared memory上のaligned `uint32_t`を`std::atomic<uint32_t>`として操作する。
対応architectureでのlock-free性、process-shared動作、C++ object model上の扱いには
可搬性上の制約がある。

実運用platformはLinux x86_64/aarch64とし、対象toolchainとarchitectureで32-bit atomicが
lock-freeであることを確認する必要がある。異なるarchitecture、標準library、ABI間での
shared memory相互運用は保証しない。

### 12.5 TTLは配送保証ではない

短いTTLはslot回収を早める一方、遅延messageがgeneration mismatchでdropされる可能性を
高める。長いTTLは配送猶予を増やす一方、message drop時にslotを長く占有する。

TTLは最大処理遅延、QoS、publish rate、pool sizeに合わせて設定する必要がある。

### 12.6 subscription countは瞬間値

ROS graphのsubscription countはpublish前後に変化し得る。新規Subscriber、切断、QoS不一致、
middleware上のdropにより、`pending_count`と実際のlease取得数は一致しない場合がある。

このprotocolは不一致を配送保証で補正せず、pendingのTTL回収とgenerationによる安全なdropで
処理する。

---

## 13. Failure handling

| failure | behavior |
| --- | --- |
| reusable slotがない | Publisher acquire失敗 |
| GPU resource初期化失敗 | 作成済みresourceをrollback |
| ready event記録失敗 | slotは未commitのまま残り、destructorでcancel |
| ready記録前のdescriptor要求 | 失敗 |
| ROS message構築またはpublish前の失敗 | destructorでcancel |
| commit忘れ | destructorでcancelを試行。遅延Subscriberはdropし得る |
| generation mismatch | Subscriber acquire失敗 |
| refcnt overflow | Subscriber acquire失敗 |
| stale pending | TTL条件を満たす場合にPublisherが回収 |
| Subscriber crash | refcnt leak。自動回収しない |
| SHM capacity不一致 | reservationをrollbackしてPublisher acquire失敗。並行再初期化自体は非対応 |

---

## 14. 検証対象

protocol変更時は少なくとも次をテストする。

- active leaseとpendingによるslot再利用防止
- generation mismatch拒否
- Subscriber acquireによるpending減少
- Publisher reserveとSubscriber acquireの競合
- 複数Publisher threadによるreserve競合
- TTL回収と次回acquire
- generationを照合したcancel
- reset後の未commit reservation cancel
- `PublishSlot`のmove、commit、cancel、destructor状態遷移
- ready event記録前のdescriptor拒否
- GPU backend部分初期化失敗時のrollback

Publisher restart、Subscriber process crash、generation wraparound、異種platform間atomicは
現在のunit testだけでは安全な回復を保証しない。これらは既知制約として扱う。
