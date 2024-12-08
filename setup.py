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
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kulka136',
    maintainer_email='kulka136@msu.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lane_assist = lane_assist_package.lane_assist:main', 
            'lane_regression = lane_assist_package.lane_regression:main', 
            'lane_detection = lane_assist_package.lane_detection:main',
        ],
    },
)
