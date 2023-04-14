from setuptools import setup

package_name = 'drive_train'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ndt',
    maintainer_email='thoennesn@gmail.com',
    description='Test package for the Terraformer Rover drivetrain',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'controller_node = drive_train.controller_node:main',
            'new_controller_node = drive_train.new_controller_node:main',
            'driver_node = drive_train.driver_node:main',
            'new_driver_node = drive_train.new_driver_node:main'
        ],
    },
)
