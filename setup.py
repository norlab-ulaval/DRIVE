import os
from glob import glob
from setuptools import setup

package_name = 'doughnut_calib'

setup(
    name='doughnut_calib',
    version='0.0.2',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files.
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
        # Include all param files
        (os.path.join('share', package_name), glob('config/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Dominic Baril',
    maintainer_email='dominic.baril@norlab.ulaval.ca',
    description='The doughnut calibrator node',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'doughnut_calib_node = doughnut_calib.doughnut_calib:main',
            'doughnut_keyboard_node = doughnut_calib.doughnut_keyboard:main',
        ],
    },
)
