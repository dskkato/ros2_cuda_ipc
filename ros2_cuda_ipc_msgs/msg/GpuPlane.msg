uint64 size_bytes           # Total size of this plane in bytes (>0)
uint64 pitch_bytes          # Row stride in bytes; 0 when data is tightly packed
uint8[64] ipc_mem_handle    # cudaIpcMemHandle_t (64 bytes); open via cudaIpcOpenMemHandle for read-only access
