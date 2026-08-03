# VMM+FD Unix socket path and CUDA IPC ready-event handle
string vmm_socket_path
uint8[64] event_handle

# Direct block locator and publication identity.  The subscriber derives the
# metadata object name as /ros2_cuda_ipc_<publisher_pid>_<block_id>.
uint32 publisher_pid
uint32 device_id
uint32 block_id
uint64 uid

# Total bytes in the GPU buffer
uint64 byte_size
