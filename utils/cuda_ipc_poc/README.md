# cuda_ipc_poc

Test applications for different CUDA IPC (Inter-Process Communication) mechanisms.

## Overview

This package contains standalone test programs that demonstrate and verify different CUDA memory sharing approaches:

1. **CUDA IPC** (`cuda_ipc_producer.cu`, `cuda_ipc_consumer.cu`): Traditional CUDA IPC using `cudaIpcGetMemHandle`/`cudaIpcOpenMemHandle` and `cudaIpcGetEventHandle`/`cudaIpcOpenEventHandle`
2. **VMM-FD** (`vmm_fd_producer.cu`, `vmm_fd_consumer.cu`): Modern approach using CUDA Driver API Virtual Memory Management with POSIX file descriptor export and CUDA IPC event synchronization

These tests are intentionally kept as standalone applications with no ROS2 dependencies in the source code, making them useful for debugging IPC issues independently of ROS2.

## Build Options

### As ROS2 Package

```bash
cd /path/to/workspace
export CUDACXX=/usr/local/cuda/bin/nvcc  # Set CUDA compiler path
colcon build --packages-select cuda_ipc_poc
source install/setup.bash
```

### As Standalone CMake Project

```bash
cd utils/cuda_ipc_poc
export CUDACXX=/usr/local/cuda/bin/nvcc  # Set CUDA compiler path if needed
cmake -B build .
cmake --build build -j
```

Executables will be in `build/` directory:
- `build/cuda_ipc_producer`
- `build/cuda_ipc_consumer`
- `build/vmm_fd_producer`
- `build/vmm_fd_consumer`

Run the standalone test script:
```bash
./test_ipc.sh  # Automatically builds if needed
```

## Usage

### Standalone Mode (No ROS2 Required)

Simply run the test script from the package directory:
```bash
cd utils/cuda_ipc_poc
./test_ipc.sh
```

The script will automatically build the executables if needed and run both tests.

Or run individual executables manually:
```bash
# Build first
cmake -B build . && cmake --build build -j

# Terminal 1: Start producer
./build/cuda_ipc_producer 0

# Terminal 2: Start consumer
./build/cuda_ipc_consumer
```

### Run Individual Executables

With ROS2 (ensure you source the workspace first):
```bash
# Source ROS2 and workspace
source /opt/ros/humble/setup.bash
source install/local_setup.bash

# Terminal 1: Start producer
ros2 run cuda_ipc_poc cuda_ipc_producer 0

# Terminal 2: Start consumer
ros2 run cuda_ipc_poc cuda_ipc_consumer
```

Or for VMM-FD test:
```bash
# Terminal 1: Start producer
ros2 run cuda_ipc_poc vmm_fd_producer 0

# Terminal 2: Start consumer
ros2 run cuda_ipc_poc vmm_fd_consumer
```

### Using Launch Files

Test CUDA IPC:
```bash
source /opt/ros/humble/setup.bash
source install/local_setup.bash
ros2 launch cuda_ipc_poc cuda_ipc.launch.py
```

Test VMM-FD:
```bash
ros2 launch cuda_ipc_poc vmm_fd.launch.py
```

## Test Description

### CUDA IPC Test (cuda_ipc_producer / cuda_ipc_consumer)

- **Communication**: Named pipes (FIFOs) at `/tmp/cuda_ipc_handle.fifo` and `/tmp/cuda_ipc_done.fifo`
- **Memory Allocation**: `cudaMalloc` (Runtime API)
- **IPC Method**: `cudaIpcGetMemHandle` / `cudaIpcOpenMemHandle` and `cudaIpcGetEventHandle` / `cudaIpcOpenEventHandle`
- **Ready Synchronization**: Consumer waits on the producer's CUDA IPC event before reading the shared memory
- **Expected Behavior**: 
  - Producer allocates memory, fills with value 123
  - Consumer opens the handle, reads data, adds 7 (result: 130)
  - Producer verifies the modification

### VMM-FD Test (vmm_fd_producer / vmm_fd_consumer)

- **Communication**: Unix domain socket at `/tmp/cuda_fd_share.sock`
- **Memory Allocation**: `cuMemCreate` + `cuMemAddressReserve` + `cuMemMap` (Driver API)
- **IPC Method**: POSIX file descriptor via `SCM_RIGHTS`
- **Ready Synchronization**: Consumer waits on the producer's CUDA IPC event before reading the imported memory
- **Memory Alignment**: Uses `cuMemGetAllocationGranularity` for proper alignment
- **Expected Behavior**:
  - Producer allocates memory with proper granularity, fills with value 123
  - Consumer imports the FD, maps the memory, reads data, adds 7 (result: 130)
  - Producer verifies the modification

## Troubleshooting

### CUDA IPC Test Fails

- **Integrated GPU**: CUDA IPC typically doesn't work on integrated GPUs (error 801)
- **Different GPU architectures**: IPC requires compatible GPUs with peer access support
- **Solution**: Use VMM-FD test instead on integrated/incompatible systems

### VMM-FD Test Fails

- **Driver version**: Requires CUDA 11.2+ with driver support for Virtual Memory Management
- **cuMemImportFromShareableHandle fails**: Check CUDA driver version and GPU support

### General Issues

- Clean up stale FIFOs/sockets: `rm /tmp/cuda_ipc_*.fifo /tmp/cuda_fd_share.sock`
- Check CUDA device: Pass different device ID as argument (e.g., `0`, `1`)
- Verify GPU compatibility: Check `nvidia-smi` and CUDA samples

## Architecture

All test applications are pure CUDA code without ROS2 dependencies:
- `src/cuda_ipc/cuda_ipc_producer.cu`: CUDA IPC producer (Runtime API)
- `src/cuda_ipc/cuda_ipc_consumer.cu`: CUDA IPC consumer (Runtime API)
- `src/vmm_fd/vmm_fd_producer.cu`: VMM-FD producer (Driver API)
- `src/vmm_fd/vmm_fd_consumer.cu`: VMM-FD consumer (Driver API)
- `include/cuda_ipc_poc/cuda_ipc/ipc_msg.hpp`: CUDA IPC message definition
- `include/cuda_ipc_poc/vmm_fd/ipc_msg.hpp`: VMM-FD message definition
- `include/cuda_ipc_poc/common/cuda_check.hpp`: CUDA/Driver API error-check helpers

The CMakeLists.txt supports both ROS2 (with `ament_cmake`) and standalone builds, making these tests portable and reusable.
