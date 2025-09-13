from setuptools import setup

package_name = 'ros2_cuda_ipc_py'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dskkato',
    maintainer_email='kato.daisuke429@gmail.com',
    description='Python bindings for ROS2 CUDA IPC transport.',
    license='MIT',
)
