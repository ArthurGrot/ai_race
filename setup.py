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
            'client = ' + package_name + '.client:main',
            'client_cancel = ' + package_name + '.client_cancel:main',
            'client_not_composable = ' + package_name + '.client_not_composable:main',
            'teleop = ' + package_name + '.teleop:main',
            'racecar = ' + package_name + '.racecar:main',
        ],
    },
)
