from setuptools import find_packages, setup
from glob import glob

package_name = 'cora_moveit'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/cora_moveit/launch', glob('launch/*.launch.py')),
        ('share/cora_moveit/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='matt',
    maintainer_email='M.J.deLannoy@student.tudelft.nl',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'cora_moveit_test = cora_moveit.mover_node_test:main',
            'moveitpy_node = cora_moveit.mover_node:main',
            'keyboard_servo_node = cora_moveit.moveit_servo_test:main'
        ],
    },
)
