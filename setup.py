import os
from glob import glob
from setuptools import setup

package_name = 'drive'

setup(
    name='drive',
    version='0.1.0',
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
    install_requires=['setuptools', 'numpy', 'pandas', 'scipy',  'norlab_controllers_msgs', 'rclpy'],
    zip_safe=True,
    maintainer='Dominic Baril',
    maintainer_email='dominic.baril@norlab.ulaval.ca',
    description='The DRIVE training dataset gathering package',
    license='BSD 3.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'drive_node = drive.drive_node:main',
            'drive_maestro_node = drive.drive_maestro_node:main',
            'pose_cmds_logger_node = drive.pose_cmds_logger_node:main',
            
        ],
    },
)
