from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from ros2_cuda_ipc_msgs.msg import GpuBuffer
from ros2_cuda_ipc_msgs.srv import GpuBufferRelease
from ros2_cuda_ipc_py import GpuBufferMapper


class GpuBufferSubscriber(Node):
    """Subscriber that maps GPU buffers and notifies release."""

    def __init__(self) -> None:
        super().__init__('dummy_gpu_buffer_subscriber')
        qos = QoSProfile(depth=10)
        self.sub = self.create_subscription(
            GpuBuffer, 'gpu_buffer', self.on_msg, qos)
        self.release_client = self.create_client(
            GpuBufferRelease, 'gpu_buffer_release')
        self.mapper = GpuBufferMapper()
        self.stream = self._create_stream()

    def _create_stream(self) -> Optional[int]:
        try:
            import cupy as cp  # type: ignore
            return int(cp.cuda.Stream(non_blocking=True).ptr)
        except Exception as exc:  # pragma: no cover - defensive
            self.get_logger().warn(f'Failed to create CUDA stream: {exc}')
            return None

    def on_msg(self, msg: GpuBuffer) -> None:
        self.get_logger().info(f'Received seq_id {msg.seq_id}')
        if msg.plane_count > 0 and len(msg.planes) >= 1:
            mem_handle = bytes(msg.planes[0].ipc_mem_handle)
            try:
                self.mapper.open_memory(msg.pool_slot_id, mem_handle)
            except Exception as exc:
                self.get_logger().error(f'open_memory failed: {exc}')
                return
        try:
            evt_handle = bytes(msg.ipc_event_handle)
            self.mapper.open_event(msg.pool_slot_id, evt_handle)
            if self.stream is not None:
                self.mapper.wait_ready(msg.pool_slot_id, self.stream)
        except Exception as exc:
            self.get_logger().error(f'wait_ready failed: {exc}')
            return
        if not self.release_client.service_is_ready():
            self.release_client.wait_for_service(timeout_sec=0.5)
        req = GpuBufferRelease.Request()
        req.seq_id = msg.seq_id
        req.pool_slot_id = msg.pool_slot_id
        req.consumer_id = self.get_fully_qualified_name()
        self.release_client.call_async(req)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GpuBufferSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
