uint32 abi_version                # ABI version; increment when incompatible changes are made
string device_uuid                # Target CUDA device UUID (cudaDeviceProp::uuid); ensures same GPU
uint64 seq_id                     # Monotonic sequence number for the buffer/frame (0..2^64-1)
uint32 pool_slot_id               # Index of slot in publisher's pool [0..N-1]
uint8 plane_count                 # Number of valid entries in 'planes'; range 1-255 (typ. 1 for RGB(A), 2 for NV12)

# format enum: 1=BGR8, 2=RGBA8, 10=NV12, 11=YUV420, 20=FP16, 21=FP32, 30=PCL_XYZ
uint8 format                      # Pixel/element format code describing memory contents

# layout enum: 0=LINEAR, 1=PITCHED, 10=CHW, 11=NCHW, 20=AOS, 21=SOA
uint8 layout                      # Memory layout of each plane

uint32 width                      # Width in pixels/elements; must be > 0
uint32 height                     # Height in pixels/elements; must be > 0
uint32 channels                   # Channels per pixel/element; typical range 1-4

GpuPlane[] planes                 # Per-plane memory descriptors (length == plane_count)
uint8[64] ipc_event_handle        # cudaIpcEventHandle_t (64 bytes); open via cudaIpcOpenEventHandle and wait before use

builtin_interfaces/Time stamp     # ROS timestamp when buffer was produced
string frame_id                   # Frame of reference, follows standard ROS usage
string shm_name                   # Optional shared memory control block name; may be empty
