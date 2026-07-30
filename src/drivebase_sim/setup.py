from glob import glob

from setuptools import find_packages, setup

package_name = 'drivebase_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/worlds', glob('worlds/*.sdf')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Robot Curiosity',
    maintainer_email='bryanhsu0123@gmail.com',
    description='Gazebo Harmonic simulation of the litter-collection drivebase.',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'laserscan_to_range = drivebase_sim.laserscan_to_range:main',
            'gps_covariance = drivebase_sim.gps_covariance:main',
        ],
    },
)
