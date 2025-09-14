from setuptools import setup

package_name = 'sample_nodes_py'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dskkato',
    maintainer_email='kato.daisuke429@gmail.com',
    description='Python sample nodes for ros2_cuda_ipc.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gpu_buffer_publisher = sample_nodes_py.gpu_buffer_publisher:main',
            'gpu_buffer_subscriber = sample_nodes_py.gpu_buffer_subscriber:main',
        ],
    },
)
