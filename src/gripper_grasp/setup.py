from setuptools import find_packages, setup

package_name = 'gripper_grasp'

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
    description='Decision layer for the three finger gripper',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'grasp_pilot = gripper_grasp.grasp_pilot:main',
        ],
    },
)
