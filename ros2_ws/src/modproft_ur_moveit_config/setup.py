from setuptools import setup
import os
from glob import glob

package_name = 'modproft_ur_moveit_config'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'srdf'), glob('srdf/*.xacro')),
    ],
    install_requires=['setuptools', 'os', 'glob'],
    zip_safe=True,
    maintainer='Max Schnitzler',
    maintainer_email='Maximilian.Schnitzler@HS-Augsburg.de',
    description='UR Moveit configuration for ModProFT',
    license='confidential',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
