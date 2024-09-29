from setuptools import setup, find_packages
from glob import glob
import os

package_name = 'path_track'
MPC = 'path_track/MPC'
Stanley = 'path_track/Stanley'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, MPC, Stanley],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dn',
    maintainer_email='dn@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "mpc_tracker = path_track.mpc:main",
            "stanley_tracker = path_track.stanley:main"
        ],
    },
)
