# Subscriber CUDA IPC implementationをDriver APIへ移行する

`ros2_cuda_ipc`のSubscriber実装を、CUDA Runtime API依存を減らすためDriver APIへ移行してください。

## 背景

Python対応により、PyTorchなどと同一processで使う予定です。Runtime APIの競合を避けるため、Subscriber側のCUDA IPC処理をDriver APIへ統一します。

対象処理:

* IPC memory/eventのimport・解放
* event wait
* VMM memory操作

## ゴール

* IPC memory/event import・cleanupをDriver API化
* event waitを`cuStreamWaitEvent`へ変更
* VMM含め可能な範囲で統一
* `libcudart`非依存を確認
* 既存APIと動作維持
* Runtime streamとのinterop確認

## 対象

* `subscriber/`配下実装
* BufferView / Mapper
* cache / context utility
* test / CMake

## 対象外

Publisher側、API大変更、Python bindingなどは変更しない。

## API対応

```
cudaIpcOpenMemHandle  -> cuIpcOpenMemHandle
cudaIpcCloseMemHandle -> cuIpcCloseMemHandle
cudaIpcOpenEventHandle-> cuIpcOpenEventHandle
cudaEventDestroy      -> cuEventDestroy
cudaStreamWaitEvent   -> cuStreamWaitEvent
```

## Context

* primary contextを使用
* `cuInit`実行
* context guardでpush/pop管理
* reset禁止

## Stream

`cudaStream_t`は維持しつつ内部で`CUstream`へ変換可。

## Error

* `CUresult`使用
* `cuGetErrorString`でログ
* Runtime APIは使わない

## Ownership

* memory: `cuIpcCloseMemHandle`
* event: `cuEventDestroy`
* lease順序維持

## Cache

* 不完全resourceを残さない
* lifetime維持

## Build

* `libcudart`削除
* `libcuda`のみ依存

確認:

```
ldd <binary>
```

## Test

* 既存test維持
* Driver API unit test
* Runtime→Driver interop
* Runtime stream interop

## API互換

* public API維持
* 内部のみ変更

## 注意

* Runtime API混在禁止
* context破壊禁止
* cleanupで例外禁止

## Documentation

* Driver API使用
* libcudart非依存
* PyTorch想定

## 成果物

* Driver API化
* context utility
* error utility
* CMake更新
* interop test
* 非依存確認

## 完了条件

* Driver APIのみでIPC処理
* event wait成功
* cleanup正常
* test通過
* libcudart非依存
* API互換維持

----

レビュー結果として、Driver APIへの置換方針は概ね妥当です。ただし、マージ前に対応したい指摘が4点あります。

## 指摘事項

1. [中] 公開APIのエラー情報が失われています

[cuda_util.hpp](/home/dskkato/workspace/ros2_cuda_ipc/ros2_cuda_ipc_core/include/ros2_cuda_ipc_core/detail/cuda_util.hpp:23) の `cuda_error_from_driver()` は、すべての失敗を `cudaErrorUnknown` に変換します。

そのため、従来 `cudaEventRecord` / `cudaStreamWaitEvent` から返されていた `cudaErrorInvalidResourceHandle` などを呼び出し側が判別できなくなります。これは型は維持していても、APIの動作互換性としては後退です。

推奨:

- 対応可能な `CUresult` をRuntime側のエラーへ明示的にマッピングする。
- 少なくとも `INVALID_HANDLE`、`INVALID_CONTEXT`、`INVALID_DEVICE`、`OUT_OF_MEMORY` は区別する。
- 変換不能なものだけ `cudaErrorUnknown` にする。
- 変換テストを追加する。

2. [中] Runtime stream interopテストが実質的に存在しません

[test_driver_api.cpp](/home/dskkato/workspace/ros2_cuda_ipc/ros2_cuda_ipc_core/test/test_driver_api.cpp:42) の `EmptyViewDoesNotNeedRuntimeStreamInterop` は、イベントを持たないViewへ `nullptr` を渡し、即座に `cudaSuccess` が返る経路しか通していません。

以下は検証されていません。

- Runtime APIで作った `cudaStream_t` を `CUstream` として `cuStreamWaitEvent` に渡せること
- Runtime側primary contextとDriver側guardの共存
- 実際に記録されたIPCイベントを別プロセスで待てること
- PyTorchが初期化したprimary contextを壊さず、呼び出し前のcontextへ戻ること

推奨する最小テストは、Runtime APIでstreamを作成し、Driver APIでeventを作成・記録して `BufferView::enqueue_ready_event()` と `cudaStreamSynchronize()` が成功するケースです。このテストだけは意図的に `libcudart` とリンクして構いません。

3. [中] primary contextのretainが一度もreleaseされません

[cuda_driver_context.cpp](/home/dskkato/workspace/ros2_cuda_ipc/ros2_cuda_ipc_core/src/detail/cuda_driver_context.cpp:34) はデバイスごとに `cuDevicePrimaryCtxRetain()` したcontextをプロセス終了まで保持し、`cuDevicePrimaryCtxRelease()` を呼びません。

NVIDIAの仕様上、retainした利用者は使用終了時にreleaseする必要があります。primary contextはRuntime APIとも共有されるため、他ライブラリの参照を壊さず、自分の参照だけ解放できます。[NVIDIA Primary Context Management](https://docs.nvidia.com/cuda/archive/13.0.3/cuda-driver-api/group__CUDA__PRIMARY__CTX.html)

通常のプロセス終了では実害が見えにくいものの、Python拡張のアンロード・再ロードではretain参照が残り続ける可能性があります。今回のPython対応という動機を考えると無視しにくい点です。

推奨:

- デバイス別context registryに所有権を持たせる。
- import済みresource/cacheの解放後に、registryがretain分をreleaseする。
- `cuDevicePrimaryCtxReset()` は引き続き呼ばない。

また、[cuda_driver_context.hpp](/home/dskkato/workspace/ros2_cuda_ipc/ros2_cuda_ipc_core/include/ros2_cuda_ipc_core/detail/cuda_driver_context.hpp:29) の `previous_` は保存されるだけで使用されておらず、移行途中の名残です。push/popだけで以前のcontextは復元されるため削除できます。[NVIDIA Context Management](https://docs.nvidia.com/cuda/cuda-driver-api/group__CUDA__CTX.html)

4. [中] `cuda_error_to_string()` が潜在的なlibcudart依存とABI破壊を残しています

[cuda_util.hpp](/home/dskkato/workspace/ros2_cuda_ipc/ros2_cuda_ipc_core/include/ros2_cuda_ipc_core/detail/cuda_util.hpp:13) で、以前ライブラリ側にあった関数をinline化し、`cudaGetErrorName()` / `cudaGetErrorString()` を呼んでいます。

問題は次の2点です。

- 既存の非inlineシンボルが消えるため、ビルド済みdownstreamに対するABI互換性が失われる。
- downstreamがこの関数を使うと、公開CMake targetが `CUDA::cudart` を伝播しないためリンクに失敗する。

実際に `CUDA::cuda_driver` 相当だけでこの関数を呼ぶ最小プログラムをリンクすると、`cudaGetErrorString` と `cudaGetErrorName` が未解決になりました。

`detail` APIを互換性対象外と明示するなら削除が最も明快です。互換性を維持するなら、Runtime用utilityを別ターゲットへ分離して明示的に `CUDA::cudart` へリンクする方が安全です。

## 作業途中の名残

未追跡の [doc/instruction.md](/home/dskkato/workspace/ros2_cuda_ipc/doc/instruction.md) は実装指示書そのもので、成果物として残す文書には見えません。コミットせず削除するのが妥当です。

[doc/design.md](/home/dskkato/workspace/ros2_cuda_ipc/doc/design.md:180) にも旧実装の説明が複数残っています。

- `cudaMalloc` / `cudaIpcGetMemHandle`
- `cudaIpcOpenMemHandle` / `cudaIpcOpenEventHandle`
- `cudaStreamWaitEvent`
- 「BufferViewのハンドル解放は送信側に委ねる」という、新しいcache destructorと矛盾する記述
- Driver API化後もエラー例が `cudaErrorInvalidDevice` 等のまま

特に [424–427行付近](/home/dskkato/workspace/ros2_cuda_ipc/doc/design.md:424) は現在の「cacheがimport済みresourceを解放する」実装と矛盾しています。

## 確認できた良い点

- CUDA IPC memory/eventのimport・解放はDriver APIへ置換されています。
- `cuIpcOpenEventHandle()` でimportしたeventを `cuEventDestroy()` する所有権処理は公式仕様どおりです。[NVIDIA CUDA IPC documentation](https://docs.nvidia.com/cuda/cuda-driver-api/group__CUDA__MEM.html)
- context guardはpush/popにより呼び出し前のcontextを復元します。
- `cuDevicePrimaryCtxReset()` は呼ばれていません。
- import途中で失敗したevent、VMM mapping、allocationのロールバックは概ね適切です。
- 現在の10テストはすべて成功しました。
- 全テスト実行ファイルの `ldd` を確認し、`libcudart` 依存はなく `libcuda` のみでした。
- `git diff --check` は問題ありません。
- tracked fileに未コミット変更はありません。

結論として、Driver API移行そのものは成立しています。ただし「既存APIと動作維持」「Runtime stream interop確認」「Python環境でのcontext寿命」の完了条件はまだ満たし切れていません。特にエラー変換と実interopテストを直してからマージするのが安全です。
