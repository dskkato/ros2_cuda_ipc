# Python Subscriber

## 目的

Pythonから既存のC++ Subscriber coreを利用し、GPU payloadをコピーせずに
framework objectへ渡す。

ROS subscriptionとexecutorは`rclpy`に残し、Python callbackで受け取ったmessageを
binding経由で既存のC++ mapperへ渡す。

```text
rclpy message
    -> Python/C++ descriptor boundary
    -> C++ BufferViewMapper / ImageViewMapper
    -> Python BufferView / ImageView
    -> shared tensor metadata
        -> CuPy / DLPack framework object
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
- device pointer、ready event、metadataの提供
- view破棄時のresource cleanup

descriptorで渡すのはmetadataとhandleだけであり、GPU payload bytesはコピーしない。
node、executor、QoSの管理も`rclpy`に委ねる。

## 対象範囲

現在の中心対象は`BufferCore`、`GpuImage`、CUDA IPC、CUDA上のframework objectである。
CuPyは一つのadapterであり、framework-neutralな相互運用経路はDLPackである。
Python Publisher、任意のROS messageの自動変換、PointCloud2、CPU fallback、
自動的なstream-ordered lease releaseは含めない。

## Ownership model

mapperが返すviewは、imported resourceと`LeaseHandle`を含むnative C++ viewを
Python objectが保持する。

```text
Python view
    -> native C++ view
        -> BufferView
            -> imported resource
            -> LeaseHandle
                -> shared-memory slot lease
```

slotは、最後のnative ownerがleaseを解放するまでpublisherから再利用されない。
`close()`はそのPython viewが持つnative ownershipを解放し、viewを無効にする。

framework objectを作成する場合は、元のviewとは独立したnative ownerを持たせる。
そのため、元のPython viewを破棄または`close()`しても、framework objectが生きて
いる間はimported resourceとslot leaseが維持される。

## Framework adapterの原則

framework objectはraw device pointerだけを保持してはいけない。framework objectの
lifetimeをnative viewへ接続し、slotの所有権を一方向に拡張する。

```text
framework object
    -> retained native view
        -> imported resource
        -> LeaseHandle
            -> shared-memory slot
```

これはmemoryの所有権であり、CUDA kernelの完了通知ではない。最後のframework object
だけでなく、元のviewを含む最後のnative ownerが破棄された時点でleaseが解放される。

## Shared tensor metadata

CuPyとDLPackは、mapped native `ImageView`から一度だけ作られるprivateな
tensor metadataを共有する。metadataにはdevice pointer、CUDA device ID、dtype、
rank、shape、byte stride、element stride、byte offset、allocation boundsとretained
native ownerが含まれる。dtype、stride単位、pointer arithmetic、allocation範囲、
imported allocationのdeviceはnative側で検証される。

そのため、frameworkごとにshapeやstrideの変換を再実装せず、同じlayoutを渡す。

## CuPy adapter

現在実装するadapterは`GpuImage`からCuPy ndarrayを作る経路である。shape、strides、
dtype、device pointerをmetadataとしてCuPy objectへ渡し、payloadはコピーしない。
CuPyの`UnownedMemory.owner`には、元のPython wrapperではなくretained native
`ImageView`を設定する。

```text
CuPy ndarray
    -> MemoryPointer
        -> UnownedMemory
            -> retained native ImageView
```

`as_cupy(stream)`は、arrayを返す前にpublisherのready event waitを指定streamへ
enqueueする。streamのdeviceはimageのdeviceと一致しなければならない。CuPyは
optional runtime dependencyであり、native bindingはCuPyをimportまたはlinkしない。

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

CuPyでも同じ`ImageView`から`cupy.from_dlpack(image)`を呼べる。どちらもpayloadを
copyせず、shared tensor metadataのshape、dtype、device、non-contiguous strideを
framework objectへ渡す。`torch.from_dlpack(image)`と`cupy.from_dlpack(image)`は
current framework versionsで利用できるlegacy DLPack capsuleを既定値として受け取り、
`__dlpack__(max_version=(1, 0))`ではversioned DLPack v1.0 capsuleを返す。

DLPackのownership chainは次の通りである。

```text
framework tensor / array
    -> DLPack managed tensor または CuPy owner
        -> manager context / retained native ImageView
            -> imported CUDA resource
            -> LeaseHandle
                -> shared-memory slot lease
```

capsuleがconsumerに渡された後はmanaged-tensor deleterがretained native viewを
解放する。未consumeのcapsuleが破棄された場合もcapsule destructorが同じdeleterを
呼ぶ。capsuleは一度だけconsumeできる。deleterはPython APIやGILを使わない。

`__dlpack_device__()`はCUDA device typeとmapped device IDを返す。`stream=None`は
legacy default stream、`1`はlegacy default、`2`はper-thread default、`-1`はproducer
ready waitを要求しない特殊値、`>2`は通常のCUDA stream pointerとして扱う。`0`と
その他の負値は拒否する。通常streamはnative側でstreamのdeviceとimported allocation
のdeviceを照合する。

## CUDA同期とlifetime

producer ready eventの待機と、consumer kernelの完了は別のイベントである。

```text
producer ready event wait
    -> consumer work enqueue
    -> stream completion
    -> framework object / retained owner release
```

Python objectのlifetimeとCUDA kernelの実行期間は別である。ready event waitは
producerの書き込み完了だけを表し、DLPack objectの破棄はconsumer streamの完了を
表さない。Pythonの参照解放だけではCUDA workの完了は保証されないため、利用者は
consumer streamの処理が終わるまでframework objectを保持する必要がある。元の
`ImageView`を`close()`しても、framework objectが生きている間はleaseは保持される。

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
