from setuptools import find_packages, setup

package_name = 'agv_image_processing'

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
    maintainer='minh',
    maintainer_email='minh@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "range_detection_with_ultrasonic = agv_image_processing.range_detection_with_ultrasonic:main",
            "range_detection_with_safety = agv_image_processing.range_detection_with_safety_v2:main",
            "range_detection_update_safety = agv_image_processing.range_detection_update_safety:main",
            "controller_agv_speed = agv_image_processing.controller_agv_speed:main",  
            "controller_agv_speed_update = agv_image_processing.controller_agv_speed_update:main",  
            "controller_agv_speed_ultrasonic = agv_image_processing.controller_agv_speed_ultrasonic:main",  
            "safety_control = agv_image_processing.safety_control:main",  
            "safety_ultrasonic = agv_image_processing.safety_ultrasonic:main",  
        ],
    },
)
