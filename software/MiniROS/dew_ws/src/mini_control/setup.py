from setuptools import find_packages, setup
from glob import glob


package_name = 'mini_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*launch.py')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='neslisah',
    maintainer_email='zneslisahyilmaz@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "velocity_node = mini_control.velocity_publisher:main",
            'pipe_read = mini_control.pipe_read:main',
            'bumper = mini_control.bumper:main',
            'improved_teleop_key = mini_control.improved_teleop_key:main',
            'neopixel = mini_control.neopixel:main',
            'pipe_write = mini_control.pipe_write:main',
            'emergency_button = mini_control.emergency_button:main',
	        'camera_test = mini_control.camera_test:main',
            'battery = mini_control.battery:main',
            'distance_node = mini_control.distance_node:main',
        ],
    },
)
