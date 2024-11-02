from setuptools import find_packages, setup

package_name = 'waypoint_logger'

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
    maintainer='ebrahim',
    maintainer_email='ebrahim@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'waypoints_logger = waypoint_logger.waypoints_logger:main',
            'initial_pose_set = waypoint_logger.initial_pose_set:main',
        ],
    },
)
