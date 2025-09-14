from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from ros2_cuda_ipc_msgs.msg import GpuBuffer
from ros2_cuda_ipc_msgs.srv import GpuBufferRelease
from ros2_cuda_ipc_py import GpuBufferMapper, to_dlpack


class GpuBufferSubscriber(Node):
    """Subscriber that maps GPU buffers and converts them to DLPack."""

    def __init__(self) -> None:
        super().__init__('gpu_buffer_subscriber')
        qos = QoSProfile(depth=10)
        self._sub = self.create_subscription(
            GpuBuffer, 'gpu_buffer', self._on_msg, qos)
        self._release_client = self.create_client(
            GpuBufferRelease, 'gpu_buffer_release')
        self._mapper = GpuBufferMapper()
        self._stream = self._create_stream()

    def _create_stream(self) -> Optional[int]:
        try:
            import cupy as cp  # type: ignore
            return int(cp.cuda.Stream(non_blocking=True).ptr)
        except Exception as exc:  # pragma: no cover - defensive
            self.get_logger().warn(f'Failed to create CUDA stream: {exc}')
            return None

    def _on_msg(self, msg: GpuBuffer) -> None:
        try:
            self.get_logger().info(f'Received seq_id {msg.seq_id}')
            if msg.plane_count > 0 and len(msg.planes) >= 1:
                # Map memory using IPC handle
                try:
                    mem_handle = bytes(msg.planes[0].ipc_mem_handle)
                    self._mapper.open_memory(msg.pool_slot_id, mem_handle)
                except Exception as exc:
                    self.get_logger().error(f'open_memory failed: {exc}')
                    return
            # Open event and wait on our stream
            try:
                evt_handle = bytes(msg.ipc_event_handle)
                self._mapper.open_event(msg.pool_slot_id, evt_handle)
                if self._stream is not None:
                    self._mapper.wait_ready(msg.pool_slot_id, self._stream)
            except Exception as exc:
                self.get_logger().error(f'wait_ready failed: {exc}')
                return
            # Convert to DLPack capsule
            if msg.plane_count > 0 and len(msg.planes) >= 1:
                ptr = self._mapper.get_memory(msg.pool_slot_id)
                if ptr:
                    size = int(msg.planes[0].size_bytes)
                    try:
                        cap = to_dlpack(int(ptr), size)
                        try:
                            import cupy as cp  # type: ignore
                            arr = cp.fromDlpack(cap)
                            self.get_logger().info(
                                f'Converted to CuPy array with shape {arr.shape}')
                        except Exception:
                            self.get_logger().info('Produced DLPack capsule')
                    except Exception as exc:
                        self.get_logger().error(f'DLPack conversion failed: {exc}')
            # Notify release
            if not self._release_client.service_is_ready():
                self._release_client.wait_for_service(timeout_sec=0.5)
            req = GpuBufferRelease.Request()
            req.seq_id = msg.seq_id
            req.pool_slot_id = msg.pool_slot_id
            req.consumer_id = self.get_fully_qualified_name()
            try:
                future = self._release_client.call_async(req)
                future.result(timeout=1.0)
            except Exception as exc:
                self.get_logger().warn(f'Release service call failed: {exc}')
        except Exception as exc:  # pragma: no cover - defensive
            self.get_logger().error(f'Callback error: {exc}')


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
