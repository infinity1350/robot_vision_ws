from setuptools import find_packages, setup

package_name = 'hp60c_vision'

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
    maintainer='optimus',
    maintainer_email='daisonedwin@gmail.com',
    description='HP60C YOLO object detection with obstacle avoidance',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'yolo_obstacle_node = hp60c_vision.yolo_obstacle_node:main',
            'obstacle_avoidance_node = hp60c_vision.obstacle_avoidance:main',
        ],
    },
)