from setuptools import setup
import os
from glob import glob

package_name = 'my_nav_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='Simple A* + local navigation',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'astar_planner = my_nav_pkg.astar_planner_node:main',
            'local_nav = my_nav_pkg.local_nav_node:main',
            'debug_viz = my_nav_pkg.debug_viz_node:main',
        ],
    },
)
