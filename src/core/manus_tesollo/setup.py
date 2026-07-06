from glob import glob

from setuptools import find_packages, setup

package_name = 'manus_tesollo'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/urdf',   glob('urdf/*.urdf')),
        ('share/' + package_name + '/retargeters/configs',
            glob('manus_tesollo/retargeters/configs/*.yml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jyi48',
    maintainer_email='ljhtheman@gmail.com',
    description='Manus glove to Tesollo DG5F-M retargeting node',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'manus_tesollo_node = manus_tesollo.manus_tesollo_node:main',
        ],
    },
)
