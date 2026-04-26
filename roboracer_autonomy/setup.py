from setuptools import setup
import os
from glob import glob

package_name = 'roboracer_autonomy'

setup(
    name=package_name,
    version='2.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='RoboRacer Team',
    maintainer_email='team@example.com',
    description='Competition-ready autonomy stack for AutoDRIVE RoboRacer - Disparity Extender + Pure Pursuit',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'autonomy_node = roboracer_autonomy.autonomy_node:main',
        ],
    },
)
