# VMM+FD memory payload and CUDA IPC ready-event handle (opaque byte arrays)
uint8[64] mem_handle
uint8[64] event_handle

# Shared memory name for buffer metadata/refcount management
string shm_name

# Identity of the publisher initialisation that owns this buffer
uint8[16] publisher_instance_id

# Identification of the GPU buffer slot
uint32 device_id
uint32 slot_id
uint32 generation

# Total bytes in the GPU buffer
uint64 byte_size
