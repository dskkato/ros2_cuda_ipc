# PR 06: V2 examples と documentation cutover

## 目的

V2 APIを実際の利用経路で検証し、fanout exampleとPython examplesをV2 contractの受け入れ基準にする。このPRでbuffer lifecycle文書を設計上の正本へ切り替え、V1文書をlegacy/historyとして残す。

## example移行

`multi_process_image_fanout` にV2のpublisher/subscriber経路を追加・切替する。

- GPU image topicは `/fanout/v2/image_gpu`、関連preview/status topicは `/fanout/v2/...` を使う。
- publisherは `try_acquire_for_write()`、`BufferWrite::commit(stream)`、`GpuImageV2.descriptor` を使う。
- C++ subscriberはV2 `BufferMapper` またはV2 typed mapperのstream-bound readを使い、同じCUDA streamへkernel/copyをenqueueする。
- Python CuPy/PyTorch examplesは `ros2_cuda_ipc_py.v2.ImageMapper` を使う。
- TensorRT等のraw pointer consumerはV2 `BufferMapper.map(descriptor, stream)` と `ReadHandle.device_ptr` を使うことをREADMEに記載する。
- V1 example/topicをV2へbridgeしない。必要ならV1/V2 launch profileを別々に起動する。

## documentation cutover

- `doc/buffer_lifecycle.md` をslot lifecycleの正本として `doc/design.md` から参照する。
- `doc/lease_protocol.md` をV1 terminology/current implementationまたはhistoryとして明示する。
- `doc/python-subscriber-implementation.md` とroot READMEをV1/V2 API・topic・Python import pathの境界に合わせて更新する。
- `doc/refactoring-migration.md` と `doc/refactoring/v2/` を実行履歴として維持する。

## rollout と受け入れ条件

- V1/V2 message type、C++ namespace、Python import path、fanout topicが同じworkspaceで共存する。
- V1/V2間に暗黙のconversionやdual publishがない。
- V2 launchでpublisher、preview、encoder-like、inference-likeが別processとして動き、GPU imageをhost payload copyなしでfanoutする。previewだけがfull device-to-host copyを行う。
- V2 build/testはCPU-only環境で適切にskipし、CUDA環境ではkernel smoke test、publisher/subscriber lifecycle、DLPack framework integrationを実行する。
- V2 failure logsはstable codeを含み、quarantine/reset、version mismatch、stale publicationが運用文書から追跡できる。

## 範囲外

- V1/V2 bridge、dual publish、旧subscriberの自動移行。
- subscriber crashのreader回収、heartbeat、timeout reclamation。
- metrics registry、ROS diagnostics publisher、外部監視基盤の追加。
