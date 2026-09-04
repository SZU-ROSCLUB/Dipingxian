from setuptools import setup
import os
from glob import glob

package_name = 'mylio'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
    ],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='Odometry test node',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mylio_node = mylio.mylio:main',
        ],
    },
)