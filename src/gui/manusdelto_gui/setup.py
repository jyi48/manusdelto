from setuptools import find_packages, setup

package_name = 'manusdelto_gui'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='J. Yi',
    maintainer_email='jaehyun.yi@samsung.com',
    description='Minimal PySide6 GUI for the Manus + DG5F standalone test rig',
    license='MIT',
    entry_points={
        'console_scripts': [
            'manusdelto_gui_node = manusdelto_gui.manusdelto_gui_node:main',
        ],
    },
)
