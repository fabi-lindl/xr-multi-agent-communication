from setuptools import find_packages, setup

package_name = 'rosrouter'

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
            ['package.xml']
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fabi-lindl',
    maintainer_email='maintainer@maintainer.com',
    description='Routes traffic between different system agents.',
    license='Apache Licence 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rosrouter_node = rosrouter.rosrouter_node:main'
        ],
    },
)
