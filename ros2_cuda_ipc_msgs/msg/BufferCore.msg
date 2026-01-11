# Memory backend type
uint8 CUDA_IPC=0
uint8 VMM_FD=1
uint8 backend

# CUDA IPC handles (opaque byte arrays)
uint8[64] mem_handle
uint8[64] event_handle

# Shared memory name for lease/refcount management
string shm_name

# Identification of the GPU buffer slot
uint32 device_id
uint32 slot_id
uint32 generation

# Total bytes in the GPU buffer
uint64 byte_size
