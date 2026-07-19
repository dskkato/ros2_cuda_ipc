# 今後の課題

現行の保証とcallerが守るべき契約は
[`lease_protocol.md`](lease_protocol.md)を参照すること。

## Publisher shutdown時のdrain

正常終了時はpublisher所有のGPU resourceを破棄し、instance固有のPOSIX SHM名をunlinkする。
Subscriber queue内のmessageや処理中のCUDA workをdrainするprotocolはまだ実装していない。
graceful shutdownを保証するには、publish停止、queue drain、active lease完了待ち、resource破棄の
順序とtimeoutを定義する必要がある。

## Crash後のorphan SHM

Publisherがdestructorを通らず終了するとUUID付きSHM objectが残り得る。再起動後は別の
`publisher_instance_id`とSHM名を使うため、新instanceとの状態混同は発生しない。
一方、orphanを列挙・検証・期限付きで削除するtoolまたは運用手順は今後追加する必要がある。
