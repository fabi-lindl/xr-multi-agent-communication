from setuptools import find_packages, setup

package_name = 'rosagent'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            [
                'package.xml',
                'launch/rosagent_launch.py'
            ]
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fabi-lindl',
    maintainer_email='maintainer@maintainer.com',
    description='ROS TCP endpoint for the ROS-Unity communication system.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'test_node = rosagent.quicktests.test_node:main',
            'rosagent_node = rosagent.rosagent_node:main'
        ],
    },
)
