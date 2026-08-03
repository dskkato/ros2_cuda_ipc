# BufferView 設計の履歴

> Historical document. 現行 API の仕様ではない。現行設計は
> [design.md](design.md) を参照する。

## 背景

以前の Subscriber API は、`BufferCore` を import した結果をコピー可能な
`BufferView` として返していた。`BufferView` は device pointer、byte size、block の
識別情報、imported resource、`LeaseHandle` をまとめて保持し、画像と点群の view は
これを内包していた。

このモデルでは、producer ready event の wait は `enqueue_ready_event(stream)` を
呼び出す利用者の責務であり、view の生存期間が publication lease の生存期間でもあった。
そのため、GPU work を enqueue した直後に view が破棄されると、consumer work が完了する
前に block が再利用され得るという制約があった。

## `ReadHandle` への移行理由

この制約を解消するため、Subscriber の read を次のように変更した。

```text
旧: BufferView の生存期間
    -> lease の生存期間

現: ReadHandle の破棄
    -> consumer completion event を stream に record
    -> deferred queue が event 完了を確認
    -> lease と imported resource を解放
```

移行後は、ready wait と consumer completion の両方が core の lifecycle に組み込まれ、
非同期 CUDA framework と組み合わせた場合にも ownership の境界を明示できる。

## 旧 API からの対応

| 旧 API | 現行 API |
| --- | --- |
| `BufferViewMapper::map(BufferCore)` | `BufferMapper::map(BufferCore, CUstream)` |
| `BufferView` | move-only `ReadHandle` |
| `ImageView` / `PointCloud2View` が `BufferView` を保持 | typed view が `ReadHandle` を保持 |
| caller が `enqueue_ready_event()` を呼ぶ | `BufferMapper` が map 時に ready wait を enqueue |
| view 破棄で lease release | completion event 後に deferred release |

この文書に残す情報は移行理由と対応表に限定する。旧型の完全な疑似ヘッダーや旧 mapper
の実装方針は、現行設計の参照元になりやすいため再掲しない。
