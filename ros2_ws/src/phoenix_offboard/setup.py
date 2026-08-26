from glob import glob
from setuptools import find_packages, setup

package_name = 'phoenix_offboard'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='PhoenixDrone port',
    maintainer_email='dev@px4.io',
    description='ROS 2 Jazzy equivalents of PX4-PhoenixDrone offboard demos.',
    license='BSD-3-Clause',
    entry_points={'console_scripts': [
        'position_demo = phoenix_offboard.position_demo:main',
        'rectangle_demo = phoenix_offboard.rectangle_demo:main',
        'attitude_demo = phoenix_offboard.attitude_demo:main',
        'phoenix_position_controller = phoenix_offboard.phoenix_position_controller:main',
        'phoenix_controller = phoenix_offboard.phoenix_controller:main',
    ]},
)
