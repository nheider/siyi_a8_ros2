from setuptools import setup
import os
from glob import glob

package_name = 'siyi_a8_ros2'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='ROS2 interface for SIYI A8 Mini camera and gimbal',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'siyi_camera_node = siyi_a8_ros2.siyi_camera_node:main',
        'gimbal_tf_broadcaster = siyi_a8_ros2.gimbal_tf_broadcaster:main',
    ],
},
)
