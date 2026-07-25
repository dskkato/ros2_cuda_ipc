"""High-level Python views and DLPack exports."""

import ctypes
import operator

from . import _native
from ._descriptor import buffer_core_descriptor, gpu_image_descriptor


MappingError = _native.MappingError
_UINTPTR_MAX = (1 << (ctypes.sizeof(ctypes.c_void_p) * 8)) - 1


def _debug_info(native_view):
    """Return private native diagnostics for view representations."""
    try:
        return native_view._debug_info()
    except AttributeError:
        # Keep lightweight test doubles and older private native objects usable
        # without making diagnostics part of the public view contract.
        return {}


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
        return 0, False
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
    def device_id(self):
        return self._native.device_id

    def close(self):
        self._native.close()

    def __repr__(self):
        return (
            "BufferView("
            f"valid={self.valid!r}, "
            f"debug={_debug_info(self._native)!r})"
        )

    def __enter__(self):
        return self

    def __exit__(self, _exc_type, _exc_value, _traceback):
        self.close()


class ImageView:
    """Lease-backed image metadata and framework-neutral zero-copy exports."""

    def __init__(self, native_view):
        self._native = native_view

    @classmethod
    def _from_native(cls, native_view):
        return cls(native_view)

    @property
    def valid(self):
        return self._native.valid

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

    def __dlpack_device__(self):
        """Return the DLPack CUDA device tuple ``(device_type, device_id)``."""

        if not self.valid:
            raise MappingError("cannot export an invalid ImageView through DLPack")
        return self._native._dlpack_device()

    def __dlpack__(self, stream=None, max_version=None):
        """Return a zero-copy DLPack capsule for this image.

        Legacy capsules are emitted by default for compatibility with current
        CuPy and PyTorch releases. A consumer that advertises
        ``max_version >= (1, 0)`` receives the versioned DLPack v1.0 capsule.
        Unless ``stream=-1`` is requested, the producer-ready event is waited
        on the requested CUDA stream before the capsule is returned. As
        required by DLPack, ``stream=-1`` disables producer synchronization.
        The consuming framework object owns the retained native view after
        capsule consumption.
        """

        pointer, synchronize = _dlpack_stream_pointer(stream)
        versioned = _dlpack_versioned(max_version)
        return self._native._dlpack(pointer, synchronize, versioned)

    def close(self):
        self._native.close()

    def __repr__(self):
        return (
            "ImageView("
            f"valid={self.valid!r}, "
            f"shape={self.shape!r}, "
            f"strides={self.strides!r}, "
            f"dtype={self.dtype!r}, "
            f"encoding={self.encoding!r}, "
            f"frame_id={self.frame_id!r}, "
            f"debug={_debug_info(self._native)!r})"
        )

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
