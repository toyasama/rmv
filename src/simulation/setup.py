from setuptools import find_packages, setup
from glob import glob 
import os
package_name = 'simulation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob ('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='devuser',
    maintainer_email='emerytognama@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "markers = simulation.markers_pub:main",
            "tf_broadcaster = simulation.tf_broadcast:main",
        ],
    },
)
