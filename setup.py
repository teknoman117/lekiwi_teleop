from glob import glob

from setuptools import setup

package_name = 'lekiwi_teleop'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nathan Lewis',
    maintainer_email='git@nrlewis.dev',
    description='Teleoperation of the LeKiwi / SO-101 arm with MoveIt IK',
    license='MIT',
    entry_points={
        'console_scripts': [
            'arm_marker = lekiwi_teleop.arm_marker:main',
            'joy_teleop = lekiwi_teleop.joy_teleop:main',
        ],
    },
)
