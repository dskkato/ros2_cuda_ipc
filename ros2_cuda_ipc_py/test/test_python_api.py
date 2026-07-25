import copy
import gc
import struct

import pytest
from ros2_cuda_ipc_msgs.msg import GpuImage

from ros2_cuda_ipc_py import ImageMapper, ImageView, MappingError
from ros2_cuda_ipc_py import _native
from ros2_cuda_ipc_py._descriptor import gpu_image_descriptor


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


def test_dlpack_rejects_stride_not_representable_in_elements():
    _native_view, probe, descriptor = _fixture()
    del _native_view
    malformed = copy.deepcopy(descriptor)
    malformed["dtype"] = 1  # uint16
    malformed["shape"] = [1, 1, 1]
    malformed["strides"] = [1, 1, 1]
    image = ImageMapper().map(malformed)
    with pytest.raises(BufferError, match="divisible"):
        image.__dlpack__(stream=-1)
    image.close()
    del image
    gc.collect()
    assert probe.refcount() == 0


def test_view_exceeding_allocation_bounds_is_rejected():
    _native_view, _probe, descriptor = _fixture()
    invalid = copy.deepcopy(descriptor)
    invalid["shape"] = [2, 3, 4]
    invalid["strides"] = [24, 8, 2]
    with pytest.raises(ValueError, match="exceed"):
        ImageMapper().map(invalid)


def test_stale_generation_is_reported_as_mapping_error():
    _native_view, _probe, descriptor = _fixture()
    stale = copy.deepcopy(descriptor)
    stale["core"]["generation"] += 1

    with pytest.raises(MappingError, match="C\\+\\+ mapper"):
        ImageMapper().map(stale)


def test_dlpack_device_and_stream_protocol_arguments():
    class SpyNative:
        valid = True
        device_ptr = 1
        byte_size = 1
        device_id = 3
        shape = (1, 1, 1)
        strides = (1, 1, 1)
        dtype_code = 0
        encoding = ""
        frame_id = ""

        def close(self):
            self.valid = False

        def _dlpack(self, stream_ptr, synchronize, versioned):
            self.arguments = (stream_ptr, synchronize, versioned)
            return "capsule"

    native = SpyNative()
    image = ImageView._from_native(native)
    assert image.__dlpack_device__() == (2, 3)
    assert image.__dlpack__(stream=-1, max_version=(1, 0)) == "capsule"
    assert native.arguments == (0, False, True)
    assert image.__dlpack__(stream=2) == "capsule"
    assert native.arguments == (2, True, False)
    max_uintptr = (1 << (struct.calcsize("P") * 8)) - 1
    assert image.__dlpack__(stream=max_uintptr) == "capsule"
    assert native.arguments == (max_uintptr, True, False)

    with pytest.raises(ValueError, match="ambiguous"):
        image.__dlpack__(stream=0)
    with pytest.raises(ValueError, match="-1 or positive"):
        image.__dlpack__(stream=-2)
    with pytest.raises(TypeError, match="stream"):
        image.__dlpack__(stream=object())


def test_dlpack_device_copy_and_keyword_only_arguments():
    class SpyNative:
        valid = True

        def _dlpack_device(self):
            return (2, 3)

        def _dlpack(self, stream_ptr, synchronize, versioned):
            self.arguments = (stream_ptr, synchronize, versioned)
            return "capsule"

        def close(self):
            self.valid = False

    native = SpyNative()
    image = ImageView._from_native(native)

    assert image.__dlpack__() == "capsule"
    assert image.__dlpack__(copy=False) == "capsule"
    assert image.__dlpack__(dl_device=(2, 3)) == "capsule"
    assert image.__dlpack__(dl_device=[2, 3]) == "capsule"

    with pytest.raises(BufferError, match="copying"):
        image.__dlpack__(copy=True)
    with pytest.raises(BufferError, match="cross-device"):
        image.__dlpack__(dl_device=(2, 4))
    with pytest.raises(BufferError, match="cross-device"):
        image.__dlpack__(dl_device=(1, 0))
    with pytest.raises(TypeError):
        image.__dlpack__(None)


def test_unconsumed_dlpack_capsule_releases_lease():
    native_view, probe, _descriptor = _fixture()
    image = ImageView._from_native(native_view)
    del native_view

    capsule = image.__dlpack__(stream=-1)
    image.close()
    assert probe.refcount() == 1
    del capsule
    gc.collect()
    assert probe.refcount() == 0


def test_dlpack_versioned_capsule_has_standard_name():
    native_view, probe, _descriptor = _fixture()
    image = ImageView._from_native(native_view)
    del native_view
    capsule = image.__dlpack__(stream=-1, max_version=(1, 0))
    assert "dltensor_versioned" in repr(capsule)
    del capsule, image
    gc.collect()
    assert probe.refcount() == 0


def test_torch_from_dlpack_is_zero_copy_and_retains_lease():
    try:
        import torch
    except ImportError:
        pytest.skip("PyTorch is not installed")
    if not torch.cuda.is_available():
        pytest.skip("CUDA device is not available")

    native_view, probe, _descriptor = _fixture()
    image = ImageView._from_native(native_view)
    del native_view
    tensor = torch.from_dlpack(image)
    assert tensor.is_cuda
    assert tensor.data_ptr() == image.device_ptr
    assert tuple(tensor.shape) == image.shape
    assert tuple(tensor.stride()) == (12, 4, 1)
    assert str(tensor.dtype) == "torch.uint8"

    image.close()
    assert probe.refcount() == 1
    del image
    gc.collect()
    assert probe.refcount() == 1
    del tensor
    gc.collect()
    assert probe.refcount() == 0


def test_torch_dlpack_capsule_cannot_be_consumed_twice():
    try:
        import torch
    except ImportError:
        pytest.skip("PyTorch is not installed")
    if not torch.cuda.is_available():
        pytest.skip("CUDA device is not available")

    native_view, probe, _descriptor = _fixture()
    image = ImageView._from_native(native_view)
    del native_view
    capsule = image.__dlpack__()
    tensor = torch.from_dlpack(capsule)
    with pytest.raises((RuntimeError, ValueError)):
        torch.from_dlpack(capsule)
    image.close()
    del image, tensor, capsule
    gc.collect()
    assert probe.refcount() == 0


def test_cupy_from_dlpack_is_zero_copy_and_retains_lease_when_available():
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
    image = ImageView._from_native(native_view)
    del native_view
    array = cp.from_dlpack(image)
    assert array.data.ptr == image.device_ptr
    assert array.shape == image.shape
    assert array.strides == image.strides
    assert str(array.dtype) == "uint8"

    image.close()
    assert probe.refcount() == 1
    del image
    gc.collect()
    assert probe.refcount() == 1
    del array
    gc.collect()
    assert probe.refcount() == 0
