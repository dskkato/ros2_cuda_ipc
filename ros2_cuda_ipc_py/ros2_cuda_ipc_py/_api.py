"""High-level Python views and CuPy conversion."""

import operator

from . import _native
from ._descriptor import buffer_core_descriptor, gpu_image_descriptor


MappingError = _native.MappingError


def _stream_pointer(cupy, stream):
    stream_object = stream if stream is not None else cupy.cuda.get_current_stream()
    try:
        pointer = operator.index(getattr(stream_object, "ptr", stream_object))
    except TypeError as exc:
        raise TypeError(
            "stream must be a CuPy stream or an integer pointer"
        ) from exc
    if pointer < 0:
        raise ValueError("CUDA stream pointer must be non-negative")
    return stream_object, pointer


class BufferView:
    """Python owner of one native subscriber buffer view."""

    def __init__(self, native_view):
        self._native = native_view

    @classmethod
    def _from_native(cls, native_view):
        return cls(native_view)

    @property
    def valid(self):
        return self._native.valid

    @property
    def device_ptr(self):
        return self._native.device_ptr

    @property
    def byte_size(self):
        return self._native.byte_size

    @property
    def device_id(self):
        return self._native.device_id

    @property
    def slot_id(self):
        return self._native.slot_id

    @property
    def generation(self):
        return self._native.generation

    def wait(self, stream):
        """Enqueue the producer-ready event on a CuPy stream."""

        import cupy as cp

        stream_object, pointer = _stream_pointer(cp, stream)
        self._native.wait(pointer)
        return stream_object

    def close(self):
        self._native.close()

    def __enter__(self):
        return self

    def __exit__(self, _exc_type, _exc_value, _traceback):
        self.close()


class ImageView:
    """Lease-backed image metadata and zero-copy CuPy conversion."""

    _DTYPE_NAMES = {
        0: "uint8",
        1: "uint16",
        2: "float16",
        3: "float32",
        4: "float64",
        5: "int16",
        6: "int32",
        7: "uint32",
    }

    def __init__(self, native_view):
        self._native = native_view

    @classmethod
    def _from_native(cls, native_view):
        return cls(native_view)

    @property
    def valid(self):
        return self._native.valid

    @property
    def device_ptr(self):
        return self._native.device_ptr

    @property
    def byte_size(self):
        return self._native.byte_size

    @property
    def device_id(self):
        return self._native.device_id

    @property
    def slot_id(self):
        return self._native.slot_id

    @property
    def generation(self):
        return self._native.generation

    @property
    def shape(self):
        return self._native.shape

    @property
    def strides(self):
        return self._native.strides

    @property
    def dtype_code(self):
        return self._native.dtype_code

    @property
    def dtype(self):
        try:
            return self._DTYPE_NAMES[self.dtype_code]
        except KeyError as exc:  # The native mapper rejects this before return.
            raise ValueError(
                "unsupported ros2_cuda_ipc image dtype "
                f"{self.dtype_code}"
            ) from exc

    @property
    def encoding(self):
        return self._native.encoding

    @property
    def frame_id(self):
        return self._native.frame_id

    def wait(self, stream):
        """Enqueue the producer-ready event on ``stream``; this is non-blocking."""

        import cupy as cp

        stream_object, pointer = _stream_pointer(cp, stream)
        self._native.wait(pointer)
        return stream_object

    def as_cupy(self, stream=None):
        """Return a zero-copy CuPy ndarray with an independent native owner.

        ``stream`` defaults to CuPy's current stream. The ready-event wait is
        enqueued before the array is returned. The caller must keep the array
        alive until all asynchronous work using it has completed; closing this
        ImageView does not release the array's native lease early.
        """

        import cupy as cp

        if not self.valid:
            raise MappingError("cannot create a CuPy view from an invalid image")

        stream_object, pointer = _stream_pointer(cp, stream)
        stream_device = getattr(stream_object, "device_id", None)
        if stream_device is not None:
            stream_device = int(stream_device)
            if stream_device == -1:
                # CuPy uses -1 for a stream associated with the current
                # device, so resolve that sentinel before comparing devices.
                stream_device = int(cp.cuda.runtime.getDevice())
            if stream_device != self.device_id:
                raise ValueError(
                    "CuPy stream device does not match the mapped image device "
                    f"({stream_device} != {self.device_id})"
                )

        # Give the array an independent native owner. The retained view shares
        # the imported allocation and lease with this view, but keeps them alive
        # when the caller explicitly closes the original ImageView.
        self._native.wait(pointer)
        native_owner = self._native._retain()
        with cp.cuda.Device(self.device_id):
            memory = cp.cuda.UnownedMemory(
                self.device_ptr,
                self.byte_size,
                native_owner,
                self.device_id,
            )
            memory_pointer = cp.cuda.MemoryPointer(memory, 0)
            return cp.ndarray(
                shape=tuple(self.shape),
                dtype=cp.dtype(self.dtype),
                memptr=memory_pointer,
                strides=tuple(self.strides),
            )

    def close(self):
        self._native.close()

    def __enter__(self):
        return self

    def __exit__(self, _exc_type, _exc_value, _traceback):
        self.close()


class BufferViewMapper:
    """Reusable mapper for ``BufferCore`` descriptors or ROS messages."""

    def __init__(self):
        self._native = _native.BufferViewMapper()

    def map(self, message):
        try:
            native_view = self._native.map(buffer_core_descriptor(message))
        except RuntimeError as exc:
            raise MappingError(str(exc)) from exc
        return BufferView._from_native(native_view)


class ImageMapper:
    """Reusable mapper for ``GpuImage`` messages or descriptors."""

    def __init__(self):
        self._native = _native.ImageMapper()

    def map(self, message):
        try:
            native_view = self._native.map(gpu_image_descriptor(message))
        except RuntimeError as exc:
            raise MappingError(str(exc)) from exc
        return ImageView._from_native(native_view)
