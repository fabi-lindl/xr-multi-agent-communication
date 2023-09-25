from setuptools import find_packages, setup

package_name = 'demo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fabi-lindl',
    maintainer_email='maintainer@maintainer.com',
    description='Demo package for the VR infrastructure.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_publisher = demo.image_publisher:main',
            'image_subscriber = demo.image_subscriber:main',
            'webcam_publisher = demo.webcam_publisher:main',
            'pose_subscriber = demo.pose_subscriber:main'
        ],
    },
)
