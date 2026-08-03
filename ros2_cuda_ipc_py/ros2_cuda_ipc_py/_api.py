"""High-level Python views and DLPack exports."""

import ctypes
import operator

from . import _native
from ._descriptor import buffer_core_descriptor, gpu_image_descriptor


MappingError = _native.MappingError
_UINTPTR_MAX = (1 << (ctypes.sizeof(ctypes.c_void_p) * 8)) - 1


def _dlpack_stream_pointer(stream):
    """Normalize the standard CUDA DLPack stream values.

    DLPack reserves ``-1`` for "producer must not synchronize", ``1`` for
    the legacy default stream, and ``2`` for the per-thread default stream.
    Zero is deliberately rejected because it is ambiguous in the Python
    protocol.  ``None`` has legacy-default-stream semantics.
    """

    if stream is None:
        return 1, True

    try:
        pointer = operator.index(stream)
    except TypeError:
        try:
            pointer = operator.index(getattr(stream, "ptr"))
        except (AttributeError, TypeError) as exc:
            raise TypeError(
                "DLPack stream must be an integer pointer or an object with a "
                "ptr attribute"
            ) from exc

    if pointer == -1:
        raise ValueError(
            "DLPack stream=-1 is not supported because automatic completion "
            "requires a consumer stream"
        )
    if pointer == 0:
        raise ValueError("CUDA DLPack stream pointer 0 is ambiguous")
    if pointer < -1:
        raise ValueError("CUDA DLPack stream pointer must be -1 or positive")
    if pointer > _UINTPTR_MAX:
        raise ValueError("CUDA DLPack stream pointer is outside uintptr_t")
    return pointer, True


def _dlpack_versioned(max_version):
    if max_version is None:
        # Legacy is the compatibility default for current PyTorch and CuPy.
        return False
    if not isinstance(max_version, (tuple, list)) or len(max_version) != 2:
        raise TypeError("max_version must be a (major, minor) pair")
    try:
        major = operator.index(max_version[0])
        minor = operator.index(max_version[1])
    except TypeError as exc:
        raise TypeError("max_version must contain integer versions") from exc
    if major < 0 or minor < 0:
        raise ValueError("max_version values must be non-negative")
    # DLPack 1.0 changed the managed-tensor ABI.  This producer supports the
    # v1.0 versioned ABI and the pre-1.0 legacy ABI.
    return major >= 1


class ReadHandle:
    """Python owner of one stream-bound subscriber GPU read."""

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

    def close(self):
        self._native.close()

    def __enter__(self):
        return self

    def __exit__(self, _exc_type, _exc_value, _traceback):
        self.close()


class ImageReadHandle:
    """A validated image projection that owns one :class:`ReadHandle`."""

    def __init__(self, native_view):
        self._native = native_view

    @classmethod
    def _from_native(cls, native_view):
        return cls(native_view)

    @classmethod
    def from_message(cls, message, read):
        """Consume a stream-bound read lease and attach image metadata.

        ``read`` is invalid after this call. Its consumer stream must be the
        stream used by all downstream CUDA work; this early-bound path is not
        suitable when a DLPack framework chooses that stream at export time.
        Use :class:`ImageMapper` for DLPack late binding.
        """

        try:
            native_view = _native.ImageReadHandle.from_message(
                gpu_image_descriptor(message), read._native
            )
        except RuntimeError as exc:
            raise MappingError(str(exc)) from exc
        return cls._from_native(native_view)

    @property
    def valid(self):
        return self._native.valid

    @property
    def byte_size(self):
        return self._native.byte_size

    @property
    def device_id(self):
        return self._native.device_id

    @property
    def shape(self):
        return self._native.shape

    @property
    def strides(self):
        return self._native.strides

    @property
    def dtype(self):
        return self._native.dtype

    @property
    def encoding(self):
        return self._native.encoding

    @property
    def frame_id(self):
        return self._native.frame_id

    def close(self):
        self._native.close()

    def __enter__(self):
        return self

    def __exit__(self, _exc_type, _exc_value, _traceback):
        self.close()


class BufferMapper:
    """Reusable mapper for stream-bound ``BufferCore`` reads."""

    def __init__(self):
        self._native = _native.BufferMapper()

    def map(self, message, stream):
        stream_ptr, _synchronize = _dlpack_stream_pointer(stream)
        try:
            native_view = self._native.map(
                buffer_core_descriptor(message), stream_ptr
            )
        except RuntimeError as exc:
            raise MappingError(str(exc)) from exc
        return ReadHandle._from_native(native_view)


class DLPackImage:
    """One-shot, late-binding CUDA DLPack producer."""

    def __init__(self, native_view):
        self._native = native_view

    @property
    def valid(self): return self._native.valid
    @property
    def byte_size(self): return self._native.byte_size
    @property
    def device_id(self): return self._native.device_id
    @property
    def shape(self): return self._native.shape
    @property
    def strides(self): return self._native.strides
    @property
    def dtype(self): return self._native.dtype
    @property
    def encoding(self): return self._native.encoding
    @property
    def frame_id(self): return self._native.frame_id

    def __dlpack_device__(self):
        return self._native._dlpack_device()

    def __dlpack__(
        self, *, stream=None, max_version=None, dl_device=None, copy=None
    ):
        current_device = self.__dlpack_device__()
        if dl_device is not None and tuple(dl_device) != current_device:
            raise BufferError("cross-device DLPack export is not supported")
        if copy is True:
            raise BufferError("copying DLPack export is not supported")
        pointer, synchronize = _dlpack_stream_pointer(stream)
        return self._native._dlpack(
            pointer, synchronize, _dlpack_versioned(max_version)
        )

    def close(self): self._native.close()


class ImageMapper:
    """Create late-binding DLPack image producers from GpuImage messages."""

    def __init__(self):
        self._native = _native.ImageMapper()

    def map(self, message):
        try:
            return DLPackImage(self._native.map(gpu_image_descriptor(message)))
        except RuntimeError as exc:
            raise MappingError(str(exc)) from exc
