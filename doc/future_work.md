# 今後の課題

現行の保証とcallerが守るべき契約は
[`buffer_metadata_protocol.md`](buffer_metadata_protocol.md)を参照すること。

## Publisher shutdown時のdrain

正常終了時はpublisher所有のGPU resourceを破棄し、instance固有のPOSIX SHM名をunlinkする。
Subscriber queue内のmessageや処理中のCUDA workをdrainするprotocolはまだ実装していない。
graceful shutdownを保証するには、publish停止、queue drain、active buffer reference完了待ち、resource破棄の
順序とtimeoutを定義する必要がある。

## Crash後のorphan SHM

Publisherがdestructorを通らず終了するとblock metadata SHM objectが残り得る。初期化時には
`/dev/shm`を走査し、owner PIDが存在しない `/ros2_cuda_ipc_<pid>_<block_id>` をcleanupする。
PID再利用時のstale descriptorは新しいrandomized UID baseとuid検証で拒否する。
