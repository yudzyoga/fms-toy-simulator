from setuptools import find_packages, setup

package_name = 'py_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['launch/launcher.py']),
        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yudzyoga',
    maintainer_email='yudsyoga@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'barcode_manager = py_robot.barcode_manager:main',
            'door_handle_manager = py_robot.door_handle_manager:main',
            'emergency_button_manager = py_robot.emergency_button_manager:main',
            'stack_light_manager = py_robot.stack_light_manager:main',
            'robot_manager = py_robot.robot_manager:main',
            'ui_manager = py_robot.ui_manager:main',
        ],
    },
)
