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
    maintainer='jcjcjc0705',
    maintainer_email='jimmy910705@gmail.com',
    description='Palm camera stream from the board',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'palm_stream = gripper_sensors.palm_stream:main',
        ],
    },
)
