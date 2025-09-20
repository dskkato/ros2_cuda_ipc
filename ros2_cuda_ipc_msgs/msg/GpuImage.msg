std_msgs/Header header

# Element format stored in the buffer
uint8 dtype

# {rows, cols, channels}
uint32[3] shape

# Byte stride for each dimension {row, col, channel}
uint64[3] strides

# Underlying GPU buffer metadata
ros2_cuda_ipc_msgs/BufferCore core

# Optional human-readable encoding (empty if unspecified)
string encoding
