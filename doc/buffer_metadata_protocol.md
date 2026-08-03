# GPU Block Metadata Protocol

## 1. Ownership model

`Each GPU block owns exactly one shared BlockMetadata object.` Pool state is
publisher-local and is not part of the IPC protocol.

```text
GpuBufferPool
  ├─ GpuBufferBlock ─ GPU allocation + BlockMetadata
  ├─ GpuBufferBlock ─ GPU allocation + BlockMetadata
  └─ GpuBufferBlock ─ GPU allocation + BlockMetadata
```

For a block, the POSIX object is derived by both sides:

```text
/ros2_cuda_ipc_<publisher_pid>_<block_id>
  BlockMetadata {
    atomic<uint64_t> uid;
    atomic<uint32_t> refcount;
    atomic<uint64_t> publish_timestamp_us;
  }
```

The object is exactly one `BlockMetadata`; there is no pool header, capacity,
or metadata array. `block_id` is allocated from a process-wide counter, so two
pool instances in one process cannot collide.

## 2. Descriptor

The wire identity is:

```cpp
struct BlockDescriptor {
  uint32_t publisher_pid;
  uint32_t block_id;
  uint64_t uid;
  // device_id, byte_size, VMM-FD socket, and ready-event handle
};
```

`publisher_pid + block_id` locates the shared metadata and publisher block.
`uid` identifies the current publication and rejects stale descriptors. The
shared-memory name and pool identity are not wire fields.

## 3. Publisher lifecycle

1. `GpuBufferManager::acquire_for_publish()` selects a reusable block.
2. The publisher claims its metadata refcount and assigns a new non-zero UID.
3. GPU work is written to that block.
4. The ready event is recorded.
5. The reservation is committed and its refcount is released.
6. The descriptor containing the new UID is sent through ROS.

During reservation, the high bit of the shared refcount is an internal
publisher-claim marker. Subscribers cannot acquire a block while it is being
overwritten. A committed publication is reusable only when its refcount is
zero and the 100 ms grace period has elapsed.

## 4. Subscriber lifecycle

```text
receive BlockDescriptor
  -> derive /ros2_cuda_ipc_<pid>_<block_id>
  -> attach BlockMetadata
  -> verify uid
  -> increment refcount
  -> verify uid again
  -> import/map GPU allocation
  -> wait for ready event
```

If either UID check fails, the descriptor is stale and is rejected. If the
metadata cache may contain an old mapping, the subscriber invalidates that
Block identity, reattaches, and checks the UID again before importing CUDA
resources. The cache key is `(publisher_pid, block_id)`, not a pool key.

The `ReadHandle` owns the acquired reference until consumer work and the
completion event have finished. Publisher and subscriber process crashes can
leave a refcount behind; the publisher removes its named shared objects during
normal reset and failed initialization cleanup.

## 5. Restart and stale-message behavior

Block IDs are never reused within a process. A new publisher initialization
gets new block IDs and new initial UIDs. If a stale object survives a crash,
`O_EXCL` prevents accidental takeover; if an object is recreated after cleanup,
the descriptor UID still has to match. A PID-reuse scenario therefore cannot
make an old descriptor valid unless both the locator and the current UID match.
