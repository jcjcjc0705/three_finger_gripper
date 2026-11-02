from setuptools import find_packages, setup

package_name = 'dynamixel_tools'

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
    description='dxl_cli, dxl_debug and the board_bridge they talk to',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'dxl_cli = dynamixel_tools.dxl_cli:main',
            'dxl_debug = dynamixel_tools.dxl_debug:main',
            'board_bridge = dynamixel_tools.board_bridge:main',
        ],
    },
)
