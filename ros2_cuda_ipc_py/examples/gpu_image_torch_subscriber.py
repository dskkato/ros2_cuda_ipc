#!/usr/bin/env python3
"""Subscribe to GpuImage and consume it through the DLPack protocol."""

import argparse

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from ros2_cuda_ipc_msgs.msg import GpuImage

from ros2_cuda_ipc_py import ImageMapper, MappingError

try:
    import torch
except ImportError as exc:
    torch = None
    _TORCH_IMPORT_ERROR = exc


class GpuImageTorchSubscriber(Node):
    def __init__(self, topic):
        super().__init__("gpu_image_torch_subscriber")
        self._mapper = ImageMapper()
        self._subscription = self.create_subscription(
            GpuImage,
            topic,
            self._on_image,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE),
        )

    def _on_image(self, message):
        try:
            image = self._mapper.map(message)
            tensor = torch.from_dlpack(image)
            consumer_stream = torch.cuda.current_stream(device=image.device_id)
            with torch.cuda.stream(consumer_stream):
                # Replace this with the application model or CUDA operation.
                result = tensor
            consumer_stream.synchronize()
            self.get_logger().info(
                f"received shape={tuple(result.shape)} dtype={result.dtype} "
                f"device={result.device}"
            )
        except (MappingError, RuntimeError, TypeError, ValueError) as exc:
            self.get_logger().warning(f"skipping GPU image: {exc}")


def main():
    if torch is None:
        raise SystemExit(
            "PyTorch is required for gpu_image_torch_subscriber.py. "
            "Install a CUDA-enabled PyTorch package that matches the "
            "installed CUDA runtime."
        ) from _TORCH_IMPORT_ERROR

    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", default="/fanout/image_gpu")
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = GpuImageTorchSubscriber(args.topic)
    try:
        rclpy.spin(node)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
