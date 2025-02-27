from setuptools import find_packages, setup

package_name = 'vehicle_controller'

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
    maintainer='tahir',
    maintainer_email='tahirifdn@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "imu_logger= vehicle_controller.imu_logger:main",
            "self_control= vehicle_controller.self_control:main",
            "circular_motion= vehicle_controller.circular_motion:main"
        ],
    },
)
