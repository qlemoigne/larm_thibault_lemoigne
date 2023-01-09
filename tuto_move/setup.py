from setuptools import setup

package_name = 'tuto_move'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'eviter_obstacle = tuto_move.eviter_obstacle:main',
            'turn_left_45 = tuto_move.turn_left_45:main',
            'turn_right_45 = tuto_move.turn_right_45:main',
            'rear_0.5m = tuto_move.read_0.5m:main',
            'scan_echo = tuto_move.scan_echo:main'
        ],
    },
)
