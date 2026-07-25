import copy
import gc
import sys
import types

import pytest
from ros2_cuda_ipc_msgs.msg import GpuImage

from ros2_cuda_ipc_py import ImageMapper, ImageView, MappingError
from ros2_cuda_ipc_py import _native
from ros2_cuda_ipc_py._descriptor import gpu_image_descriptor


class _FakeStream:
    def __init__(self, ptr=0x1234, device_id=0):
        self.ptr = ptr
        self.device_id = device_id


class _FakeDevice:
    def __init__(self, device_id):
        self.device_id = device_id

    def __enter__(self):
        return self

    def __exit__(self, _exc_type, _exc_value, _traceback):
        return False


class _FakeUnownedMemory:
    def __init__(self, ptr, size, owner, device_id):
        self.ptr = ptr
        self.size = size
        self.owner = owner
        self.device_id = device_id


class _FakeMemoryPointer:
    def __init__(self, memory, offset):
        self.memory = memory
        self.offset = offset


class _FakeArray:
    def __init__(self, shape, dtype, memptr, strides):
        self.shape = tuple(shape)
        self.dtype = dtype
        self.memptr = memptr
        self.strides = tuple(strides)


def _fake_cupy(monkeypatch, current_stream=None):
    current_stream = current_stream or _FakeStream()
    module = types.ModuleType("cupy")
    module.cuda = types.SimpleNamespace(
        get_current_stream=lambda: current_stream,
        Device=lambda device_id: _FakeDevice(device_id),
        UnownedMemory=_FakeUnownedMemory,
        MemoryPointer=_FakeMemoryPointer,
    )
    module.dtype = lambda value: value
    module.ndarray = _FakeArray
    monkeypatch.setitem(sys.modules, "cupy", module)
    return module


def _fixture():
    return _native._make_test_image()


def test_extension_imports_and_native_mapper_preserves_metadata():
    native_view, probe, descriptor = _fixture()
    view = ImageMapper().map(descriptor)

    assert native_view.valid
    assert view.valid
    assert view.shape == (2, 3, 4)
    assert view.strides == (12, 4, 1)
    assert view.dtype == "uint8"
    assert view.encoding == "rgba8"
    assert view.frame_id == "test_frame"
    assert probe.refcount() == 2

    view.close()
    del view, native_view
    gc.collect()
    assert probe.refcount() == 0


def test_generated_rclpy_message_crosses_the_descriptor_boundary():
    _native_view, probe, descriptor = _fixture()
    message = GpuImage()
    message.header.frame_id = descriptor["header"]["frame_id"]
    message.dtype = descriptor["dtype"]
    message.shape = descriptor["shape"]
    message.strides = descriptor["strides"]
    message.encoding = descriptor["encoding"]
    message.core.backend = descriptor["core"]["backend"]
    message.core.mem_handle = descriptor["core"]["mem_handle"]
    message.core.event_handle = descriptor["core"]["event_handle"]
    message.core.shm_name = descriptor["core"]["shm_name"]
    message.core.publisher_instance_id = descriptor["core"][
        "publisher_instance_id"
    ]
    message.core.device_id = descriptor["core"]["device_id"]
    message.core.slot_id = descriptor["core"]["slot_id"]
    message.core.generation = descriptor["core"]["generation"]
    message.core.byte_size = descriptor["core"]["byte_size"]

    converted = gpu_image_descriptor(message)
    assert len(converted["core"]["mem_handle"]) == 64
    mapped = ImageMapper().map(message)
    assert mapped.shape == (2, 3, 4)
    assert mapped.frame_id == "test_frame"
    mapped.close()
    del mapped, _native_view
    gc.collect()
    assert probe.refcount() == 0


def test_invalid_metadata_is_a_python_value_error():
    _native_view, _probe, descriptor = _fixture()
    invalid = copy.deepcopy(descriptor)
    invalid["shape"] = [2, 3]

    with pytest.raises(ValueError, match="shape"):
        ImageMapper().map(invalid)


def test_stale_generation_is_reported_as_mapping_error():
    _native_view, _probe, descriptor = _fixture()
    stale = copy.deepcopy(descriptor)
    stale["core"]["generation"] += 1

    with pytest.raises(MappingError, match="C\\+\\+ mapper"):
        ImageMapper().map(stale)


def test_cupy_owner_keeps_lease_until_array_release(monkeypatch):
    _fake_cupy(monkeypatch)
    native_view, probe, _descriptor = _fixture()
    view = ImageView._from_native(native_view)
    del native_view

    array = view.as_cupy(_FakeStream(ptr=0xCAFE))
    assert array.memptr.memory.ptr == view.device_ptr
    assert array.memptr.memory.owner is not view
    assert array.memptr.memory.owner.valid
    assert probe.refcount() == 1

    del view
    gc.collect()
    assert probe.refcount() == 1

    del array
    gc.collect()
    assert probe.refcount() == 0


def test_cupy_owner_keeps_lease_after_view_close(monkeypatch):
    _fake_cupy(monkeypatch)
    native_view, probe, _descriptor = _fixture()
    view = ImageView._from_native(native_view)
    del native_view

    array = view.as_cupy(_FakeStream(ptr=0xCAFE))
    array_owner = array.memptr.memory.owner

    view.close()
    assert not view.valid
    assert array_owner.valid
    assert probe.refcount() == 1

    del view
    gc.collect()
    assert probe.refcount() == 1

    del array_owner
    del array
    gc.collect()
    assert probe.refcount() == 0


def test_real_cupy_external_view_keeps_lease_when_available():
    try:
        import cupy as cp
    except ImportError:
        pytest.skip("CuPy is not installed")

    try:
        if cp.cuda.runtime.getDeviceCount() == 0:
            pytest.skip("CUDA device is not available")
    except cp.cuda.runtime.CUDARuntimeError as exc:
        pytest.skip(f"CUDA runtime is unavailable: {exc}")

    native_view, probe, _descriptor = _fixture()
    view = ImageView._from_native(native_view)
    del native_view
    stream = cp.cuda.Stream(non_blocking=True)
    array = view.as_cupy(stream)
    assert array.data.ptr == view.device_ptr
    assert array.shape == (2, 3, 4)
    assert array.strides == (12, 4, 1)
    assert probe.refcount() == 1

    del view
    gc.collect()
    assert probe.refcount() == 1
    del array
    gc.collect()
    stream.synchronize()
    assert probe.refcount() == 0


def test_wait_uses_the_requested_cupy_stream(monkeypatch):
    _fake_cupy(monkeypatch)

    class SpyNative:
        valid = True
        device_ptr = 1
        byte_size = 1
        device_id = 0
        slot_id = 0
        generation = 0
        shape = (1, 1, 1)
        strides = (1, 1, 1)
        dtype_code = 0
        encoding = "mono8"
        frame_id = ""

        def __init__(self):
            self.waited_on = []

        def wait(self, stream_ptr):
            self.waited_on.append(stream_ptr)

        def close(self):
            self.valid = False

        def _retain(self):
            return self

    native = SpyNative()
    ImageView._from_native(native).as_cupy(_FakeStream(ptr=0xBEEF))
    assert native.waited_on == [0xBEEF]
