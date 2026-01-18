#!/usr/bin/env python3
"""
Launch file for testing CUDA IPC (using cudaIpcGetMemHandle/cudaIpcOpenMemHandle).
Starts producer and consumer processes.
"""

from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, Shutdown, RegisterEventHandler
from launch.event_handlers import OnProcessExit
import os


def generate_launch_description():
    """
    Launch cuda_ipc_producer and cuda_ipc_consumer with a delay.
    
    The producer creates a FIFO and waits for the consumer to connect.
    This is a one-shot test that exits after completion.
    """
    
    # Start producer first
    producer = ExecuteProcess(
        cmd=['ros2', 'run', 'ros2_cuda_ipc_test', 'cuda_ipc_producer', '0'],
        output='screen',
        name='cuda_ipc_producer'
    )
    
    # Consumer process
    consumer = ExecuteProcess(
        cmd=['ros2', 'run', 'ros2_cuda_ipc_test', 'cuda_ipc_consumer'],
        output='screen',
        name='cuda_ipc_consumer'
    )
    
    # Start consumer with a small delay to ensure producer is ready
    consumer_delayed = TimerAction(
        period=2.0,
        actions=[consumer]
    )
    
    # Shutdown when producer exits (consumer finishes first, then producer)
    shutdown_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=producer,
            on_exit=[Shutdown(reason='CUDA IPC test completed')]
        )
    )
    
    return LaunchDescription([
        producer,
        consumer_delayed,
        shutdown_handler,
    ])
