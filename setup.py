import os
from glob import glob
from setuptools import setup

package_name = 'ai_race'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Arthur Grot',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Examples of action clients using rclpy.',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop_gamepad = ' + package_name + '.teleop_gamepad:main',
            'motor_jetbot = ' + package_name + '.motor_jetbot:main',
            'motor_jetracer = ' + package_name + '.motor_jetracer:main',
            'display = ' + package_name + '.display:main',
            'camera_pub = ' + package_name + '.camera_pub:main',
            'camera_sub = ' + package_name + '.camera_sub:main',
            'line_follower = ' + package_name + '.line_follower:main',
        ],
    },
)
