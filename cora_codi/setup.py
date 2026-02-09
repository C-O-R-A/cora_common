from setuptools import find_packages, setup
from glob import glob

package_name = 'cora_codi'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/cora_codi/config', glob('config/*.yaml')),
        ('share/cora_codi/launch', glob('launch/*.launch.py'))
    ],
    install_requires=[
        'setuptools',
        'codi @ git+https://github.com/C-O-R-A/CoDI.git',
    ],
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
            'codi_node = cora_codi.codi:main',
        ],
    },
)
