from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'run_car_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'params'), glob('params/*.yaml')),        
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='Intelligent Car Competition Control Package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'run_car_node = run_car_pkg.run_car_node:main',
        ],
    },
)
