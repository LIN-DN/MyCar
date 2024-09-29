from setuptools import setup

package_name = 'path_plan'
CurvesGenerator = 'path_plan/CurvesGenerator'
map = 'path_plan/map'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, CurvesGenerator, map],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            "path_planner = path_plan.planner:main"
        ],
    },
)
