# Format constants
uint8 FORMAT_BGR8=1
uint8 FORMAT_RGBA8=2
uint8 FORMAT_NV12=10
uint8 FORMAT_YUV420=11
uint8 FORMAT_FP16=20
uint8 FORMAT_FP32=21
uint8 FORMAT_PCL_XYZ=30

# Layout constants
uint8 LAYOUT_LINEAR=0
uint8 LAYOUT_PITCHED=1
uint8 LAYOUT_CHW=10
uint8 LAYOUT_NCHW=11
uint8 LAYOUT_AOS=20
uint8 LAYOUT_SOA=21

# Update the constants above when introducing new formats or layouts.

uint32 abi_version                # ABI version; increment when incompatible changes are made
string device_uuid                # Target CUDA device UUID (cudaDeviceProp::uuid); ensures same GPU
uint64 seq_id                     # Monotonic sequence number for the buffer/frame (0..2^64-1)
uint32 pool_slot_id               # Index of slot in publisher's pool [0..N-1]
uint8 plane_count                 # Number of valid entries in 'planes'; range 1-255 (typ. 1 for RGB(A), 2 for NV12)

uint8 format                      # Pixel/element format code describing memory contents

uint8 layout                      # Memory layout of each plane

uint32 width                      # Width in pixels/elements; must be > 0
uint32 height                     # Height in pixels/elements; must be > 0
uint32 channels                   # Channels per pixel/element; typical range 1-4

GpuPlane[] planes                 # Per-plane memory descriptors (length == plane_count)
uint8[64] ipc_event_handle        # cudaIpcEventHandle_t (64 bytes); open via cudaIpcOpenEventHandle and wait before use

builtin_interfaces/Time stamp     # ROS timestamp when buffer was produced
string frame_id                   # Frame of reference, follows standard ROS usage
string shm_name                   # Optional shared memory control block name; may be empty
