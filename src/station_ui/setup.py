from setuptools import find_packages, setup

package_name = 'station_ui'

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
    maintainer='sriman',
    maintainer_email='srimans@umd.edu',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joystick_command = station_ui.joystick_command:main',
            'camera_display = station_ui.camera_display:main'
        ],
    },
)
