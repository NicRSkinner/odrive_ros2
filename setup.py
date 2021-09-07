from setuptools import setup
import os
from glob import glob

package_name = 'odrive_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools', 'odrive'],
    zip_safe=True,
    maintainer='Nick Skinner',
    maintainer_email='nicholas.skinner95@gmail.com',
    description='ODrive ROS2 node',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
		'odrive = odrive_ros2.odrive_node:main'
        ],
    },
)
