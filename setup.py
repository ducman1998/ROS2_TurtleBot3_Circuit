import os 
from glob import glob 
from setuptools import find_packages, setup

package_name = 'turtlebot3_commander'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'materials'), glob('materials/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='johnvo',
    maintainer_email='ducman1998@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "start_controler = turtlebot3_commander.controller:main",
            "start_visualizer = turtlebot3_commander.visualizer:main"
        ],
    },
)
