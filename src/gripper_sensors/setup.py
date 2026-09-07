from setuptools import find_packages, setup

package_name = 'gripper_sensors'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='Palm TOF driver',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'tof_node = gripper_sensors.tof_node:main',
            'tof_occlusion = gripper_sensors.tof_occlusion:main',
            'gripper_monitor = gripper_sensors.gripper_monitor:main',
        ],
    },
)
