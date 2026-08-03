"""Conversion from generated ROS messages to the stable native descriptor."""

from collections.abc import Mapping


def _field(value, name):
    if isinstance(value, Mapping):
        try:
            return value[name]
        except KeyError as exc:
            raise TypeError(f"message descriptor is missing field {name!r}") from exc
    try:
        return getattr(value, name)
    except AttributeError as exc:
        raise TypeError(f"message is missing field {name!r}") from exc


def _sequence(value):
    # The native boundary deliberately receives ordinary Python sequences. ROS
    # generated fixed arrays are metadata, not the GPU payload, so this copy is
    # small and keeps the binding independent of rosidl Python internals.
    try:
        return list(value)
    except TypeError as exc:
        raise TypeError("message fixed-array field must be iterable") from exc


def _header_descriptor(header):
    stamp = _field(header, "stamp")
    return {
        "stamp": {
            "sec": _field(stamp, "sec"),
            "nanosec": _field(stamp, "nanosec"),
        },
        "frame_id": _field(header, "frame_id"),
    }


def buffer_core_descriptor(message):
    """Return the descriptor accepted by the pybind11 extension."""

    if isinstance(message, Mapping):
        return dict(message)
    return {
        "vmm_socket_path": _field(message, "vmm_socket_path"),
        "event_handle": _sequence(_field(message, "event_handle")),
        "shm_name": _field(message, "shm_name"),
        "publisher_instance_id": _sequence(
            _field(message, "publisher_instance_id")
        ),
        "device_id": _field(message, "device_id"),
        "slot_id": _field(message, "slot_id"),
        "generation": _field(message, "generation"),
        "byte_size": _field(message, "byte_size"),
    }


def gpu_image_descriptor(message):
    """Convert a ``GpuImage`` ROS message without touching its GPU payload."""

    if isinstance(message, Mapping):
        descriptor = dict(message)
        if "core" in descriptor:
            descriptor["core"] = buffer_core_descriptor(descriptor["core"])
        if isinstance(descriptor.get("header"), Mapping):
            header = dict(descriptor["header"])
            if isinstance(header.get("stamp"), Mapping):
                header["stamp"] = dict(header["stamp"])
            descriptor["header"] = header
        return descriptor
    return {
        "header": _header_descriptor(_field(message, "header")),
        "dtype": _field(message, "dtype"),
        "shape": _sequence(_field(message, "shape")),
        "strides": _sequence(_field(message, "strides")),
        "core": buffer_core_descriptor(_field(message, "core")),
        "encoding": _field(message, "encoding"),
    }
