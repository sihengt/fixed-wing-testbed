from glob import glob

from setuptools import setup

package_name = 'fw_planner'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'planner = fw_planner.planner_node:main',
            'plot_flight = fw_planner.plot_flight:main',
            'bench_planners = fw_planner.bench_planners:main',
            'bench_planners3d = fw_planner.bench_planners3d:main',
        ],
    },
)
