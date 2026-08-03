# VMM+FD Unix socket path and CUDA IPC ready-event handle
string vmm_socket_path
uint8[64] event_handle

# Direct identity of the GPU block and its current publication
uint32 publisher_pid
uint32 block_id
uint64 uid

# CUDA resource description for this publication
uint32 device_id

# Total bytes in the GPU buffer
uint64 byte_size
