#!/usr/bin/env python3
"""
Launch file for testing VMM-FD based CUDA memory sharing.
Starts producer and consumer processes using Driver API Virtual Memory Management.
"""

from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, Shutdown, RegisterEventHandler
from launch.event_handlers import OnProcessExit


def generate_launch_description():
    """
    Launch vmm_fd_producer and vmm_fd_consumer with a delay.
    
    The producer creates a Unix domain socket and waits for the consumer to connect.
    This is a one-shot test that exits after completion.
    """
    
    # Start producer first
    producer = ExecuteProcess(
        cmd=['ros2', 'run', 'ros2_cuda_ipc_test', 'vmm_fd_producer', '0'],
        output='screen',
        name='vmm_fd_producer'
    )
    
    # Consumer process
    consumer = ExecuteProcess(
        cmd=['ros2', 'run', 'ros2_cuda_ipc_test', 'vmm_fd_consumer'],
        output='screen',
        name='vmm_fd_consumer'
    )
    
    # Start consumer with a small delay to ensure producer is ready and listening
    consumer_delayed = TimerAction(
        period=2.0,
        actions=[consumer]
    )
    
    # Shutdown when producer exits (consumer finishes first, then producer)
    shutdown_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=producer,
            on_exit=[Shutdown(reason='VMM-FD test completed')]
        )
    )
    
    return LaunchDescription([
        producer,
        consumer_delayed,
        shutdown_handler,
    ])
