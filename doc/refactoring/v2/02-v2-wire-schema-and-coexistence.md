# PR 02: V2 wire schema と旧新並行提供

## 目的

V1を壊さずにV2のbuffer-centric wire contractを追加する。V1/V2は同一workspaceで並行提供するが、bridgeは作らず、同世代のpublisher/subscriberだけが接続する。

## 公開wire contract

`ros2_cuda_ipc_msgs` に以下を追加する。

- `BufferDescriptor.msg`: V2のallocation/publication descriptor。
- `GpuImageV2.msg` と `GpuPointCloud2V2.msg`: V1 typed messageと同じapplication metadataを持ち、旧 `core` ではなく `descriptor: BufferDescriptor` を持つ。

`BufferDescriptor` は少なくとも次を運ぶ。

| field | 契約 |
| --- | --- |
| `protocol_version` | descriptor schemaの明示version。V2 mapperは対応version以外を拒否する。 |
| backend / memory handle / event handle | allocation identity。generationを含めない。 |
| publisher instance ID | publisher reset/reinitialization単位のidentity。 |
| device ID / slot ID / generation / byte size | slot上のpublicationとlogical bounds。 |
| metadata SHM identity | `BufferSlotMetadata` のattach先を識別する。 |

shared-memory headerにも独立した `layout_version` を置く。subscriberはdescriptorの `protocol_version` を検証してからattachし、attach時にheaderのmagic、layout version、publisher instance ID、capacityを検証する。どちらかが不一致ならmappingを拒否する。

## coexistence と名前空間

- V1 `BufferCore` / `GpuImage` / `GpuPointCloud2` は残す。
- V2 messageは同じ `ros2_cuda_ipc_msgs` packageに追加する。別packageは作らない。
- V2 C++ APIは `ros2_cuda_ipc_core::v2` 以下に置き、現namespaceのAPIをlegacyとして残す。
- V2 Python APIは `ros2_cuda_ipc_py.v2`。root moduleはlegacyのまま残す。
- V2 topicはapplication namespace内の `v2` segmentを使う。fanoutでは `/fanout/v2/image_gpu` を標準とし、関連topicも `/fanout/v2/...` に揃える。
- V1/V2間のdescriptor変換、dual publish、dual consume、暗黙のwire判別は実装しない。

## shared metadata V2

V2 `BufferSlotMetadata` はV1 `SlotMeta` と別layoutにする。少なくともgeneration、atomic `access_state`、`published_at`、`reusable_after`、quarantine状態とfailure codeを表現する。writer/readersの通常状態は `access_state` だけで表し、Published/Idleは時刻から導出する。quarantineは通常のaccess stateと混ぜない。

## テストと受け入れ条件

- V1 messageとV2 messageが同じpackageで生成・利用できる。
- V2 descriptor/version不一致、SHM layout不一致、publisher instance不一致は安全に拒否する。
- V1 publisher/V2 subscriber、および逆方向は型/topic境界で接続しないことをlaunchまたはintegration testで確認する。
- V1 package APIと既存testはgreenのまま。V2 message schema testを追加する。

## 前提

V2はfull contract breakである。ただし破壊変更の影響はV2 type/topic/namespaceへ閉じ込め、V1利用者にsource/wire変更を要求しない。
