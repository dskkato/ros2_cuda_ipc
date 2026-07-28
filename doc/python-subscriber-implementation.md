# Python Subscriber

## 目的

Pythonから既存のC++ Subscriber coreを利用し、GPU payloadをコピーせずに
framework objectへ渡す。

ROS subscriptionとexecutorは`rclpy`に残し、Python callbackで受け取ったmessageを
binding経由で既存のC++ mapperへ渡す。

```text
rclpy message
    -> Python/C++ descriptor boundary
    -> C++ BufferMapper / typed adapter
    -> Python ReadHandle / ImageView
    -> DLPack framework object
```

`ros2_cuda_ipc_py`はPythonとC++の境界をつなぐ薄いpackageであり、lease protocol、
CUDA IPC import、resource cleanupはC++ coreを利用する。

## 責務の境界

Python側は次を担当する。

- `rclpy` messageからmetadataとIPC handleをdescriptorへ変換する
- C++ mapperを呼び出し、Python viewを返す
- streamやframework objectをPython APIへ接続する

C++ coreは次を担当する。

- messageの検証とgenerationの確認
- shared-memory slotのlease取得
- CUDA memory/eventのimportとcache
- `ReadHandle`からのdevice pointer、ready wait、metadataの提供
- completion event後のresource/lease cleanup

descriptorで渡すのはmetadataとhandleだけであり、GPU payload bytesはコピーしない。
node、executor、QoSの管理も`rclpy`に委ねる。

## 対象範囲

現在の中心対象は`BufferCore`、`GpuImage`、CUDA IPC、CUDA上のframework objectである。
framework-neutralな相互運用経路はDLPackであり、CuPyもDLPack経由で利用する。
Python Publisher、任意のROS messageの自動変換、PointCloud2、CPU fallback、
自動的なstream-ordered lease releaseは含めない。

## Ownership model

`BufferMapper`が返す`ReadHandle`は、imported resourceとpublication leaseを
Python objectが保持する。DLPackのtyped adapterはmap時にはstreamへbindせず、
最初の`__dlpack__(stream)`でexport固有のread stateへownershipを移す。

```text
Python ReadHandle
    -> native ReadHandle
        -> imported resource
        -> publication lease
            -> shared-memory slot lease
```

slotは、completion event後にdeferred queueがhandle固有のleaseを解放するまで
publisherから再利用されない。通常のPython `ReadHandle`の`close()`は、そのread
stateを解放する。DLPack export後のtyped adapterは`close()`できない。

framework objectを作成する場合は、mapped objectからexport固有のnative read stateへ
ownershipを移譲する。そのため、DLPack capsuleが生きている間はimported resourceと
publication leaseが維持される。

## Framework adapterの原則

framework objectはraw device pointerだけを保持してはいけない。framework objectの
lifetimeをnative viewへ接続し、slotの所有権を一方向に拡張する。

```text
framework object
    -> retained native read state
        -> imported resource
        -> publication lease
            -> shared-memory slot
```

これはmemoryの所有権であり、CUDA kernelの完了通知ではない。capsule deleterは
bind済みconsumer streamへcompletion eventをrecordし、完了を内部queueが確認した後に
handle固有のresource参照とpublication leaseを解放する。

## DLPack adapter

`ImageView`はPython DLPack producer protocolを実装する。

```python
import torch

image = mapper.map(message)
tensor = torch.from_dlpack(image)

with torch.cuda.stream(consumer_stream):
    result = model(tensor)
consumer_stream.synchronize()
```

CuPyでは同じ`ImageView`から`cupy.from_dlpack(image)`を呼ぶ。どちらもpayloadを
copyせず、shape、dtype、device、non-contiguous strideをframework objectへ渡す。
`torch.from_dlpack(image)`と`cupy.from_dlpack(image)`は
current framework versionsで利用できるlegacy DLPack capsuleを既定値として受け取り、
`__dlpack__(max_version=(1, 0))`ではversioned DLPack v1.0 capsuleを返す。

DLPackのownership chainは次の通りである。

```text
framework tensor / array
    -> DLPack managed tensor または CuPy owner
        -> manager context / retained native read state
            -> imported CUDA resource
            -> publication lease
                -> shared-memory slot lease
```

capsuleがconsumerに渡された後はmanaged-tensor deleterがretained native read stateを
解放する。未consumeのcapsuleが破棄された場合もcapsule destructorが同じdeleterを
呼ぶ。capsuleは一度だけconsumeできる。deleterはPython APIやGILを使わない。

`__dlpack_device__()`はCUDA device typeとmapped device IDを返す。`stream=None`は
legacy default stream、`1`はlegacy default、`2`はper-thread default、正の値は
有効なCUDA stream pointerとして扱う。`-1`はcompletion管理ができないため拒否し、
`0`とその他の負値も拒否する。completion eventはcapsule公開前に作成し、その後に
ready waitを指定streamへenqueueする。capsule deleterはbind済みstreamへcompletion
eventをrecordし、event、resource参照、publication leaseを内部deferred queueへ移す。
queue workerは項目を順次`cuEventSynchronize`で待つ。metadata参照はexport前後とも
許可するが、再exportとpublication lifetimeに依存する操作、raw device pointer取得は
拒否する。

## CUDA同期とlifetime

producer ready eventの待機と、consumer kernelの完了は別のイベントである。

```text
producer ready event wait
    -> consumer work enqueue
    -> stream completion
    -> capsule deleter records completion
    -> deferred queue waits and releases handle-specific owner
```

Python objectのlifetimeとCUDA kernelの実行期間は別である。ready event waitは
producerの書き込み完了を表し、capsule deleterがconsumer streamへのcompletion
event recordを行う。利用者はframework objectと、bindしたstreamをcompletion record
完了まで保持する必要がある。import cache entryの破棄方針はcacheが管理する。

逆に、arrayを長く保持するとslot leaseも長く保持され、publisherが利用できるslot数を
圧迫する可能性がある。stream完了に合わせた自動lease releaseや、同期を伴う
context managerは今後のAPI設計課題である。

## DLPackとその他のadapter

他のDLPack-compatible frameworkを利用する場合も、基本方針は同じである。

- device pointerをコピーしない
- shape、stride、dtype、deviceを正しく伝える
- framework objectのdeleterまたはownerがnative viewを保持する
- consumerのstream semanticsを明確にする
- malformed metadataとdevice不一致を拒否する

DLPackではcapsuleのconsumeとdeleterがnative ownerの解放点になる。CuPyと同じ
ownership原則を使う。現在の実装はCUDA `GpuImage` rank 3とmessageで定義された
dtypesを対象にし、DLPack C exchange API、自動stream-completion lease release、
CPU fallback、PointCloud2は対象外である。

## エラーと実行モデル

- descriptorの型・field・layout不正はPythonの入力エラーとして扱う
- stale generation、lease取得失敗、CUDA import失敗はmapping failureとして扱う
- ready-event waitのCUDA failureはmapping failureと分けて扱う
- Python messageの読み取りにはGILが必要だが、C++ mapperの重い処理はGILを解放できる
- Python wrapperのdestructorからPython APIを呼び出さない

## 設計上の到達点

Python callbackから既存のC++ lease/resource modelを壊さずにzero-copyのGPU viewを
取得し、framework固有のobjectへ一方向にownershipを拡張できることが到達点である。
frameworkごとの使い勝手や非同期処理の完了管理は、slotの再利用条件を曖昧にしない
形で今後拡張する。
