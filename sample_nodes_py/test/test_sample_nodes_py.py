import time

import rclpy
from rclpy.executors import SingleThreadedExecutor

from sample_nodes_py.gpu_buffer_publisher import GpuBufferPublisher
from sample_nodes_py.gpu_buffer_subscriber import GpuBufferSubscriber


def test_pool_publish_subscribe_release():
    rclpy.init()
    pub = GpuBufferPublisher()
    sub = GpuBufferSubscriber()
    executor = SingleThreadedExecutor()
    executor.add_node(pub)
    executor.add_node(sub)
    try:
        pub.publish_once()
        end_time = time.time() + 5.0
        while time.time() < end_time and pub.held_slot is not None:
            executor.spin_once(timeout_sec=0.1)
        assert pub.held_slot is None, 'slot should be released by subscriber'
        slot = pub.pool.borrow()
        assert slot == 0
        pub.pool.release(slot)
    finally:
        executor.shutdown()
        pub.destroy_node()
        sub.destroy_node()
        rclpy.shutdown()
