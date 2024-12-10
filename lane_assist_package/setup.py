from setuptools import find_packages, setup

package_name = 'lane_assist_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'rclpy',             # ROS 2 Python client library
        'std_msgs',          # Standard ROS message types
        'sensor_msgs',       # ROS message types for sensors
        'geometry_msgs',     # ROS message types for geometry (Twist)
        'cv_bridge',         # OpenCV-ROS bridge
        'opencv-python-headless',  # OpenCV library for image processing
        'numpy'              # Required for array operations
    ],
    zip_safe=True,
    maintainer='kulka136',
    maintainer_email='kulka136@msu.edu',
    description='Package for lane detection and robot assistance.',
    license='Apache License 2.0',  # Replace with your chosen license
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lane_assist = lane_assist_package.lane_assist:main', 
            'lane_regression = lane_assist_package.lane_regression:main', 
            'lane_detection = lane_assist_package.lane_detection:main',
        ],
    },
)