from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'autoshield_full'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.xml')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='het',
    maintainer_email='het915@gmail.com',
    description='AutoShield full integration package with lidar processing',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'lidar_processing = autoshield_full.autoshield_lidar_processing:main',
            'straight_path = autoshield_full.autoshield_straight_path:main',
        ],
    },
)
