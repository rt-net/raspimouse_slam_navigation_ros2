from setuptools import find_packages, setup

package_name = 'raspimouse_navigation_examples'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/example.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='RT Corporation',
    maintainer_email='shop@rt-net.jp',
    description='Navigation sample package for Raspberry Pi Mouse V3',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'waypoint = raspimouse_navigation_examples.waypoint:main',
        ],
    },
)
