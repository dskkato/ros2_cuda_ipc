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
        super().__init__('dummy_gpu_buffer_publisher')
        qos = QoSProfile(depth=10)
        self.pub = self.create_publisher(GpuBuffer, 'gpu_buffer', qos)
        self.pool = GpuBufferPool(1, 4 * 1024 * 1024, True)
        self.release_srv = self.create_service(
            GpuBufferRelease, 'gpu_buffer_release', self.on_release)
        self.timer = self.create_timer(1.0, self.publish_once)
        self.held_slot: Optional[int] = None
        self.held_seq: int = 0
        self.held_since: float = 0.0
        self.lease_timeout_ms = (
            self.declare_parameter('lease_timeout_ms', 3000).value
        )

    def on_release(self, request: GpuBufferRelease.Request,
                    response: GpuBufferRelease.Response) -> GpuBufferRelease.Response:
        if self.held_slot is None:
            response.ok = False
            self.get_logger().warn('Release requested but no slot held')
            return response
        if request.pool_slot_id == self.held_slot and request.seq_id == self.held_seq:
            ok = self.pool.release(self.held_slot)
            response.ok = bool(ok)
            if ok:
                self.get_logger().info(
                    'Released slot %u for seq %u from %s',
                    request.pool_slot_id, request.seq_id, request.consumer_id)
                self.held_slot = None
                self.held_since = 0.0
            else:
                self.get_logger().warn('Release failed for slot %u', request.pool_slot_id)
        else:
            response.ok = False
            self.get_logger().warn(
                'Release mismatch: req(slot=%u, seq=%u) vs held(slot=%s, seq=%u)',
                request.pool_slot_id, request.seq_id,
                str(self.held_slot), self.held_seq)
        return response

    def publish_once(self) -> None:
        if self.held_slot is not None:
            elapsed = (time.time() - self.held_since) * 1000.0
            if elapsed > self.lease_timeout_ms:
                self.get_logger().warn(
                    'Lease timeout exceeded for slot %u (seq %u). Forcing release.',
                    self.held_slot, self.held_seq)
                self.pool.release(self.held_slot)
                self.held_slot = None
                self.held_since = 0.0
            return

        msg = GpuBuffer()
        msg.abi_version = 1
        msg.device_uuid = 'unknown'
        msg.seq_id = self.held_seq + 1
        msg.pool_slot_id = 0
        msg.format = GpuBuffer.FORMAT_BGR8
        msg.layout = GpuBuffer.LAYOUT_LINEAR
        msg.width = 0
        msg.height = 0
        msg.channels = 0
        msg.plane_count = 0
        msg.stamp = self.get_clock().now().to_msg()
        msg.frame_id = ''
        msg.shm_name = ''

        slot = self.pool.borrow()
        if slot is not None:
            plane = GpuPlane()
            plane.size_bytes = 0
            plane.pitch_bytes = 0
            plane.ipc_mem_handle = [0] * 64
            msg.planes.append(plane)
            msg.plane_count = 1
            msg.pool_slot_id = slot
            msg.ipc_event_handle = [0] * 64
            self.held_slot = slot
            self.held_seq = msg.seq_id
            self.held_since = time.time()
            self.get_logger().info(
                'Publishing seq=%u with dummy CUDA IPC handles (slot %u) [DEMO ONLY: not for production]',
                msg.seq_id, slot)
        else:
            self.get_logger().warn('Pool exhausted; publishing metadata only')

        self.pub.publish(msg)


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
