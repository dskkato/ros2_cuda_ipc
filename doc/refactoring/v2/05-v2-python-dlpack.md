# PR 05: V2 Python と DLPack ownership

## 目的

`ros2_cuda_ipc_py.v2` を追加し、V2 `GpuImageV2` のzero-copy DLPack exportを提供する。rootの `ros2_cuda_ipc_py` APIはlegacyとして残す。stream-unbound stateはPython/native adapterの内部実装であり、C++ V2 APIへ `map_for_dlpack()` 相当の公開入口は追加しない。

## Python公開API

```python
from ros2_cuda_ipc_py.v2 import BufferMapper, ImageMapper, MappingError

read = BufferMapper().map(message.descriptor, stream)  # ReadHandle
ptr = read.device_ptr                                  # TensorRT等のraw CUDA consumer

image = ImageMapper().map(message)                     # DLPack export可能なImageView
tensor = framework.from_dlpack(image)
```

- `BufferMapper.map(descriptor, stream)` はstream-bound `ReadHandle` を返す。TensorRT等は `device_ptr` を使い、同じstreamへworkをenqueueする。handleがcompletionを管理する。
- `ImageMapper.map(message)` はDLPack export可能なtyped `ImageView` を返す。metadata、device、`__dlpack_device__()`、`__dlpack__(stream=...)` を公開するが、unbound状態のraw pointerは公開しない。
- C++ V2 typed mapperは通常のstream-bound readだけを提供し、DLPack専用のunbound stateを公開しない。

## unbound / bound の契約

`ImageMapper.map()` はreader referenceとimported resourceを保持するが、GPU readはまだ開始していない。slotをpublisher再利用から保護したまま、consumer frameworkが後からstreamを指定できるようにする。

- `__dlpack__(stream)` はlayoutを検証後、ownershipをcapsule contextへ移し、ready event waitを指定streamへenqueueしてstream-bound readへ遷移する。
- 成功したexportは一度だけ。framework/capsuleのdeleterがcompletion event後のreader releaseを担う。
- export前の `close()` またはGCはGPU work未開始のためreader referenceを即時releaseする。
- bind失敗時はunbound `ImageView` へ戻し、retryまたはcloseを可能にする。slotはquarantineしない。
- `None` はlegacy default CUDA streamとして受け入れ、DLPack `stream=-1` と`0`は拒否する。

Pythonのmapping失敗は `MappingError` とし、C++の安定categoryに対応する `code` 属性を持つ。詳細なbackend原因はmessage/logへ残す。DLPack capsuleのdeleterでcompletion処理が失敗した場合は例外を返さず、slotをquarantineしてstable failure code付きstructured logを出す。

## テストと受け入れ条件

- V2 generated ROS messageとplain descriptorの両方からmetadataを変換できる。
- raw `BufferMapper` のReadHandleがTensorRT相当のstream-bound pointer lifetimeを保持する。
- `ImageView` はexport前にreader referenceを保持し、close/GCで即時releaseする。
- DLPack exportは一度だけ成功し、capsule/framework解放後かつGPU completion後にreaderをreleaseする。
- bind失敗後はretryまたはcloseできる。`None`、`-1`、`0` のstream契約を検証する。
- PyTorch/CuPy integration testは依存/CUDAがない環境でskipし、利用可能な環境ではzero-copyとlifetimeを確認する。
