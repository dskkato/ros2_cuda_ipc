#!/usr/bin/env python3
"""Subscribe to GpuImage and consume it as a zero-copy CuPy array via DLPack."""

import argparse

import cupy as cp
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from ros2_cuda_ipc_msgs.msg import GpuImage

from ros2_cuda_ipc_py import ImageMapper, MappingError


class GpuImageSubscriber(Node):
    def __init__(self, topic):
        super().__init__("gpu_image_cupy_subscriber")
        self._mapper = ImageMapper()
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self._subscription = self.create_subscription(
            GpuImage, topic, self._on_image, qos
        )

    def _on_image(self, message):
        try:
            image = self._mapper.map(message)
            array = cp.from_dlpack(image)

            # CuPy passes its current stream to the DLPack producer. The
            # synchronization before callback return makes the temporary
            # array lifetime safe for asynchronous CUDA execution.
            cp.cuda.get_current_stream().synchronize()
            self.get_logger().info(
                f"received {image.encoding or '<unspecified>'} "
                f"shape={array.shape} dtype={array.dtype} "
                f"device_ptr=0x{image.device_ptr:x}"
            )
        except (MappingError, RuntimeError, TypeError, ValueError) as exc:
            self.get_logger().warning(f"skipping GPU image: {exc}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", default="/fanout/image_gpu")
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = GpuImageSubscriber(args.topic)
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
