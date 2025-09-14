import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from ros2_cuda_ipc_msgs.msg import GpuBuffer, GpuPlane
from ros2_cuda_ipc_msgs.srv import GpuBufferRelease
from ros2_cuda_ipc_py import GpuBufferPool


class GpuBufferPublisher(Node):
    """Simple GPU buffer publisher using a tiny buffer pool."""

    def __init__(self) -> None:
        super().__init__('gpu_buffer_publisher')
        qos = QoSProfile(depth=10)
        self._pub = self.create_publisher(GpuBuffer, 'gpu_buffer', qos)
        # Prepare pool: one slot, 4 MiB, try enabling CUDA
        self._pool = GpuBufferPool(1, 4 * 1024 * 1024, True)
        self._timer = self.create_timer(1.0, self._publish_once)
        self._release_srv = self.create_service(
            GpuBufferRelease, 'gpu_buffer_release', self._on_release)
        self._held_slot: Optional[int] = None
        self._held_seq: int = 0
        self._held_since: float = 0.0
        self._lease_timeout_ms = (
            self.declare_parameter('lease_timeout_ms', 3000).value
        )

    def _on_release(self, request: GpuBufferRelease.Request,
                    response: GpuBufferRelease.Response) -> GpuBufferRelease.Response:
        try:
            if self._held_slot is None:
                response.ok = False
                self.get_logger().warn('Release requested but no slot held')
                return response
            if request.pool_slot_id != self._held_slot or request.seq_id != self._held_seq:
                response.ok = False
                self.get_logger().warn(
                    'Release mismatch: req(slot=%u, seq=%u) vs held(slot=%s, seq=%u)',
                    request.pool_slot_id, request.seq_id,
                    str(self._held_slot), self._held_seq)
                return response
            ok = self._pool.release(self._held_slot)
            response.ok = bool(ok)
            if ok:
                self.get_logger().info(
                    'Released slot %u for seq %u from %s',
                    request.pool_slot_id, request.seq_id, request.consumer_id)
                self._held_slot = None
                self._held_since = 0.0
            else:
                self.get_logger().warn('Release failed for slot %u', request.pool_slot_id)
        except Exception as exc:  # pragma: no cover - defensive
            response.ok = False
            self.get_logger().error(f'Release service error: {exc}')
        return response

    def _publish_once(self) -> None:
        try:
            # Enforce lease timeout
            if self._held_slot is not None:
                elapsed = (time.time() - self._held_since) * 1000.0
                if elapsed > self._lease_timeout_ms:
                    self.get_logger().warn(
                        'Lease timeout exceeded for slot %u (seq %u). Forcing release.',
                        self._held_slot, self._held_seq)
                    self._pool.release(self._held_slot)
                    self._held_slot = None
                    self._held_since = 0.0
                return

            slot = self._pool.borrow()
            msg = GpuBuffer()
            msg.abi_version = 1
            msg.device_uuid = 'unknown'
            msg.seq_id = self._held_seq + 1
            msg.pool_slot_id = slot or 0
            msg.format = GpuBuffer.FORMAT_BGR8
            msg.layout = GpuBuffer.LAYOUT_LINEAR
            msg.width = 0
            msg.height = 0
            msg.channels = 0
            msg.plane_count = 0
            msg.stamp = self.get_clock().now().to_msg()
            msg.frame_id = ''
            msg.shm_name = ''

            if slot is not None:
                # For demo purposes we do not export actual handles; use zeros
                plane = GpuPlane()
                plane.size_bytes = 0
                plane.pitch_bytes = 0
                plane.ipc_mem_handle = [0] * 64
                msg.planes.append(plane)
                msg.plane_count = 1
                msg.pool_slot_id = slot
                msg.ipc_event_handle = [0] * 64
                self._held_slot = slot
                self._held_seq = msg.seq_id
                self._held_since = time.time()
                self.get_logger().info(
                    'Publishing seq=%u with dummy CUDA IPC handles (slot %u)',
                    msg.seq_id, slot)
            else:
                self.get_logger().warn('Pool exhausted; publishing metadata only')

            self._pub.publish(msg)
        except Exception as exc:  # pragma: no cover - defensive
            self.get_logger().error(f'Publish error: {exc}')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GpuBufferPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
