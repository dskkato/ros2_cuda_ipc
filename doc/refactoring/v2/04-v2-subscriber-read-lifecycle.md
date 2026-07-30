# PR 04: V2 subscriber read lifecycle

## 目的

V2 subscriberを、metadata reader reference、imported resource reference、stream-bound read operationの三層へ分離する。公開C++ APIは `BufferMapper::map(descriptor, stream)` と `ReadHandle` を維持するが、V2 namespaceとresult型を使う。

## 公開API契約

```cpp
v2::subscriber::BufferMapper mapper;
v2::Expected<v2::subscriber::ReadHandle, v2::MappingError> read =
    mapper.map(message.descriptor, consumer_stream);
```

`ReadHandle` はstream-bound GPU readであり、device pointer、byte size、device ID、active reader reference、imported resource reference、consumer stream completionを所有する。callerは同じstreamへGPU workをenqueueし、handle破棄がcompletion eventをrecordするまでstreamを有効に保つ。

`MappingError` は利用者の対応が異なる安定categoryだけを公開する。最低限、`invalid_descriptor`、`unsupported_version`、`stale_publication`、`quarantined`、`resource_unavailable`、`stream_sync_failed` を扱う。backend内部の詳細はdiagnostic messageとstructured logに残す。

V2はC++17を維持するため、結果はproject-owned `ros2_cuda_ipc_core::v2::Expected<T, E>` で表す。C++23 `std::expected` やthird-party expected型を公開ABIにしない。

## mapping順序とcache

1. descriptorのprotocol versionと値域を検証する。
2. mapper所有のmetadata cacheからmappingを取得/attachし、SHM layoutとpublisher instanceを検証する。
3. current generationを検証してreader referenceを取得し、generationを再検証する。
4. reader保護下で、mapper所有のimported-resource cacheをallocation identityで取得/importする。
5. producer ready eventのwaitをconsumer streamへenqueueする。
6. すべて成功した時だけstream-bound `ReadHandle` を返す。途中失敗ではreader referenceをreleaseする。

resource cache keyはpublisher instance、backend、device、memory/event identityであり、slot IDとgenerationを含めない。generationはpublication freshnessだけを表す。metadata cacheとresource cacheはどちらも `BufferMapper` 所有で、mapper破棄または明示resetで解放する。cache entryの生存はactive reader referenceの生存を意味しない。

## deferred release と失敗

handle破棄時、completion event、reader reference、imported resourceはprocess-lifetimeの内部shared deferred queueへ移る。queueはcompletion後にreleaseするため、mapper破棄後もpending readは安全である。

completion eventのrecord/waitに失敗し安全性を証明できない場合、slotをquarantineし、stable failure code付きstructured logを出す。destructor経路では例外・process metricを公開しない。

## テストと受け入れ条件

- current generationはmap成功、stale generation・unsupported version・quarantined slotは対応する `MappingError.code` で失敗する。
- reader取得とwriter取得のraceで不正なreaderが成立しない。
- imported resource cacheはgenerationだけの変化で再importしない。
- completion event前はreader countを減らさず、event後にだけreleaseする。
- mapper破棄後もdeferred queueがpending readを完了まで保持する。
