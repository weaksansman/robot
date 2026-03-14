from setuptools import find_packages, setup

package_name = 'robot'

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
    maintainer='sussypersons',
    maintainer_email='sussypersons@gmail.com',
    description='Made by embedded 06: Package description',
    license='Apache License 2.0: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
		'UDP = robot.ESP32:main',
		'UDPS = robot.Mega:main',
		'hazmat = robot.yolo_hazmat4:main'
        ],
    },
)
