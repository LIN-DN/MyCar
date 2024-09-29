from setuptools import setup, find_packages
from glob import glob
import os

package_name = 'fusion'
setup(
    name=package_name,
    version='0.0.0',
    packages = find_packages(),  # 在setup.py的同级目录下找到所有包含__init__.py的包
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nvidia',
    maintainer_email='1449130187@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "UWB_distance = fusion.UWB:main",
            "fusion_location = fusion.main:main",
            "Error_node = fusion.errorAnalyse:main",
            "mocap_Path = fusion.rvizPlot.mocapPath:main",
            "uwb_Path = fusion.rvizPlot.uwbPath:main",
            "fusion_Path = fusion.rvizPlot.fusedPath:main"
        ],
    },
)
