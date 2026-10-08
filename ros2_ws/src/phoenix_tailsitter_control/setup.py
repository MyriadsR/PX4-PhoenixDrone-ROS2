from glob import glob

from setuptools import find_packages, setup


PACKAGE_NAME = 'phoenix_tailsitter_control'


setup(
    name=PACKAGE_NAME,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + PACKAGE_NAME]),
        ('share/' + PACKAGE_NAME, ['package.xml']),
        ('share/' + PACKAGE_NAME + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='PhoenixDrone port',
    maintainer_email='dev@px4.io',
    description='Frame-safe low-speed INDI attitude control for PhoenixDrone.',
    license='BSD-3-Clause',
    entry_points={'console_scripts': [
        'tailsitter_controller = phoenix_tailsitter_control.controller_node:main',
        'alpha_tailsitter_controller = '
        'phoenix_tailsitter_control.alpha_controller_node:main',
        'actuator_probe = phoenix_tailsitter_control.actuator_probe:main',
        'attitude_step_test = phoenix_tailsitter_control.attitude_step_test:main',
        'position_step_test = phoenix_tailsitter_control.position_step_test:main',
        'position_mission_test = '
        'phoenix_tailsitter_control.position_mission_test:main',
        'lemniscate_mission_test = '
        'phoenix_tailsitter_control.lemniscate_mission_test:main',
        'paper_trajectory_mission = '
        'phoenix_tailsitter_control.paper_trajectory_mission:main',
    ]},
)
