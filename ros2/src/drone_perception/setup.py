from setuptools import find_packages, setup

package_name = 'drone_perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        (
            'share/' + package_name,
            ['package.xml'],
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vansh',
    maintainer_email='vansh@todo.todo',
    description='Drone Rakshak perception package',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'sensor_monitor = drone_perception.sensor_monitor:main',
            'px4_monitor = drone_perception.px4_monitor:main',
            'image_buffer = drone_perception.image_buffer:main',
            'frame_processor = drone_perception.frame_processor:main',
            'frame_compare = drone_perception.frame_compare:main',
            'motion_detector = drone_perception.motion_detector:main',
            'rf_receiver = drone_perception.rf_receiver:main',
            'radar_receiver = drone_perception.radar_receiver:main',
            'radar_fusion = drone_perception.radar_fusion:main',
            'mission_node = drone_perception.mission_node:main',
        ],
    },
)
