from setuptools import setup

import os
from glob import glob

package_name = 'grp_vincent'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bot',
    maintainer_email='bot@mb6.imt-nord-europe.fr',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'scan_echo = grp_vincent.scan_echo:main',
            'camera = grp_vincent.realsense:process_img',
            'move = grp_vincent.move:main',
            'objects = grp_vincent.objects:main'
        ],
    },
)
