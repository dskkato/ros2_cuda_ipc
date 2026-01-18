#!/bin/bash
# Quick test script for CUDA IPC functionality
# This script can run standalone without ROS2

set -e

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
BUILD_DIR="${SCRIPT_DIR}/build"

echo "==================================="
echo "CUDA IPC Test Script (Standalone)"
echo "==================================="

# Check if CUDA is available
if ! command -v nvidia-smi &> /dev/null; then
    echo "ERROR: nvidia-smi not found. CUDA drivers may not be installed."
    exit 1
fi

echo "GPU Information:"
nvidia-smi --query-gpu=name,driver_version,memory.total --format=csv,noheader
echo ""

# Build if needed
if [ ! -d "$BUILD_DIR" ] || [ ! -f "$BUILD_DIR/cuda_ipc_producer" ]; then
    echo "Building test executables..."
    cd "$SCRIPT_DIR"
    export CUDACXX=/usr/local/cuda/bin/nvcc
    cmake -B build . || {
        echo "ERROR: CMake configuration failed"
        exit 1
    }
    cmake --build build -j || {
        echo "ERROR: Build failed"
        exit 1
    }
    echo "Build completed"
    echo ""
fi

# Check if executables exist
if [ ! -f "$BUILD_DIR/cuda_ipc_producer" ]; then
    echo "ERROR: cuda_ipc_producer not found in $BUILD_DIR"
    echo "Please run: cmake -B build . && cmake --build build"
    exit 1
fi

# Run individual tests
echo "==================================="
echo "Test 1: CUDA IPC (Runtime API)"
echo "==================================="
echo "Starting producer in background..."
timeout 10s "$BUILD_DIR/cuda_ipc_producer" 0 &
PROD_PID=$!
sleep 2

echo "Starting consumer..."
timeout 10s "$BUILD_DIR/cuda_ipc_consumer" || {
    echo "CUDA IPC test failed (this is expected on integrated GPUs)"
}

wait $PROD_PID 2>/dev/null || true
# Clean up any leftover FIFOs
rm -f /tmp/cuda_ipc_*.fifo

echo ""
echo "==================================="
echo "Test 2: VMM-FD (Driver API)"
echo "==================================="
echo "Starting producer in background..."
timeout 10s "$BUILD_DIR/vmm_fd_producer" 0 &
PROD_PID=$!
sleep 2

echo "Starting consumer..."
timeout 10s "$BUILD_DIR/vmm_fd_consumer" || {
    echo "VMM-FD test failed"
}

wait $PROD_PID 2>/dev/null || true
# Clean up any leftover sockets
rm -f /tmp/cuda_fd_share.sock

echo ""
echo "==================================="
echo "Tests completed"
echo "==================================="
