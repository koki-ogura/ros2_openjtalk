from setuptools import setup
import os
from glob import glob

package_name = 'ros2_openjtalk'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='koki-ogura',
    maintainer_email='koki.balian@gmail.com',
    description='ros2_openjtalk',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ros2_openjtalk_node = ros2_openjtalk.ros2_openjtalk_node:main',
            'ros2_openjtalk_test = ros2_openjtalk.ros2_openjtalk_test:main',
        ],
    },
)
