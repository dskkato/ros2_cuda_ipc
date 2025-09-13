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

uint32 abi_version
string device_uuid
uint64 seq_id
uint32 pool_slot_id
uint8 plane_count
uint8 format
uint8 layout
uint32 width
uint32 height
uint32 channels
GpuPlane[] planes
uint8[64] ipc_event_handle
builtin_interfaces/Time stamp
string frame_id
string shm_name
