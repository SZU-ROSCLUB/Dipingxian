from setuptools import setup
import os
from glob import glob

package_name = 'qrcode'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'model'), glob('qrcode/model/*')),
    ],
    install_requires=[
        'setuptools',
    ],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='QR code detection node with WeChat and pyzbar',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'qrcode_node = qrcode.qrcode:main',
        ],
    },
)