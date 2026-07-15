# 今後の課題

この文書は、現行protocolの既知制約のうち、局所的な修正では解消できず、設計変更を
伴う課題を追跡する。現行の保証とcallerが守るべき契約は
[`lease_protocol.md`](lease_protocol.md)を参照すること。

## Publisher instanceごとのshared memory identity

### 状態

設計レベルの課題。現時点では、同時に存在するPublisher instanceごとに一意な
shared memory（SHM）名をcallerが指定する必要がある。

### 問題

現在は設定されたSHM名を、そのままPOSIX SHM objectの実体名およびROS message内の
識別子として使用する。このため、複数のPublisherが同じ名前を使用した場合や、古い
Publisherのmessageが残っている間に同じ名前でPublisherを再起動した場合、次の競合が
起こり得る。

- 異なるcapacityで同じSHM headerとslot stateを初期化する。
- 既存mappingと再作成後のSHM objectが、同じ名前で異なる状態を参照する。
- 古いmessageが新しいPublisher instanceのSHMへattachする。
- SHM state、GPU memory handle、ready event handleの組み合わせが、messageを作成した
  Publisher instanceと一致しなくなる。

`SlotController`は、SHM capacity外のreservationを検出した場合にgenerationを確認して
rollbackし、ERRORを記録する。この処理はreservation leakを防ぐための防御策であり、
同名SHMの同時使用や再初期化を安全にするものではない。

### 推奨する設計

設定上のSHM名をnamespaceの接頭辞として扱い、Publisher controllerの初期化ごとに一意な
instance UUIDを生成する。POSIX SHMの実体名には、例えば
`/ros2_cuda_ipc_fanout_<uuid>`のようにUUIDを付加し、ROS messageにはこの実体名を格納する。
POSIX SHM名の移植性を保つため、先頭以外には`/`を使用しない。

この方式では、次のownershipを明確にする。

- controllerが実体SHM名を生成し、そのSHM objectを所有する。
- descriptorとROS messageは設定上の接頭辞ではなく実体SHM名を伝える。
- controllerの終了時に、自身が作成した実体SHM名だけをunlinkする。
- 実体SHM名は再利用しない。再起動後は必ず新しいUUIDを使用する。

unlink済みSHMの既存mappingはOSの参照が残る間は有効だが、遅れて到着したmessageからの
新規attachは失敗する。このfail-closedな挙動により、古いmessageが別instanceのstateへ
誤ってattachすることを防ぐ。

### 設計時に決める事項

| 項目 | 選択肢とtrade-off |
| --- | --- |
| 設定値の意味 | 現在の「実体名」から「接頭辞」へ変更すると、設定と診断出力の互換性に影響する。 |
| 実体名の公開 | logおよび診断APIから実体名を取得できるようにすると運用調査が容易になる。 |
| 正常終了時のcleanup | controller終了時に即時unlinkするか、明示的なlifecycle操作を設けるかを決める。 |
| crash後のorphan | UUID方式では名前衝突しない一方、異常終了したSHMが残る。列挙・期限付き削除toolまたは運用手順が必要になる。 |
| 互換モード | 実体名をcallerが固定する高度なoptionを残す場合、その安全条件を別途定義する必要がある。 |
| 他のIPC resource | CUDA VMM用Unix domain socketなど、instance identityを共有すべきresourceの範囲を決める。 |

### 検討した代替案

- `O_CREAT | O_EXCL`のみを使用する方法は、同時起動の衝突をfail-fastにできる。ただし、
  crash後のorphanが再起動を妨げ、unlinkして同名で再作成すると古いmessageとのABA問題が
  残る。
- owner PIDをheaderへ保存する方法は、死活判定とPID再利用を安全に扱えない。
- headerとmessageへinstance epochを追加する方法でも識別できるが、wire formatとSHM
  layoutの双方を変更し、名前自体が衝突する場合のlifecycle管理も必要になる。

### 完了条件

- 同じ設定上の接頭辞で2つのPublisher controllerを同時に作成しても、異なる実体SHM名を
  使用する。
- capacityが異なるPublisher間でheaderとslot stateが干渉しない。
- Publisher再起動前のmessageが、再起動後のinstanceへattachしない。
- 正常終了、異常終了、遅延messageに対するcleanupとfailure semanticsが文書化される。
- CUDA IPC memory backendとVMM-FD backendの双方でmulti-process testが通る。
- 実体SHM名と関連するUnix domain socket等のidentityに一貫性がある。
