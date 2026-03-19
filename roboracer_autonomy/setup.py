from setuptools import setup
import os
from glob import glob

package_name = 'roboracer_autonomy'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='OpenAI',
    maintainer_email='openai@example.com',
    description='Competition-legal autonomy stack for AutoDRIVE RoboRacer',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'autonomy_node = roboracer_autonomy.autonomy_node:main',
        ],
    },
)
