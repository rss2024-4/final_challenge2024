import os
import glob
from setuptools import setup

package_name = 'visual_servoing'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('lib/'+package_name+"/stop_detector", glob.glob(os.path.join('stop_detector/stop_detector', '*.py'))),
        ('share/stop_detector/launch', glob.glob(os.path.join('launch', '*launch.xml'))),
        ('share/stop_detector/launch', glob.glob(os.path.join('launch', '*launch.py')))
    ],  
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jospeh',
    maintainer_email='jrales@mit.edu',
    description='Visual Servoing ROS2 package',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'stop_detector = stop_detector.stop_detector:main',
        ],
    },
)
