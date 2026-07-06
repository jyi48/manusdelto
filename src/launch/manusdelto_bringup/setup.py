from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'manusdelto_bringup'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='J. Yi',
    maintainer_email='jaehyun.yi@samsung.com',
    description='Launch file for the Manus glove + DG5F hand standalone test rig',
    license='MIT',
    entry_points={
        'console_scripts': [],
    },
)
