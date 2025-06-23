from setuptools import find_packages, setup
from glob import glob

package_name = 'segurito_production'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',  glob('launch/*.py')),
        ('share/' + package_name + '/config',  glob('config/*.yaml')),
        ('share/' + package_name + '/config',  glob('config/*.pgm')),
        ('share/' + package_name + '/maps',    []),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='neverdiedooms',
    maintainer_email='neverdiedooms@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odometria = segurito_production.odometria_node:main',
            'static_laser_tf = segurito_production.static_laser_tf:main',
            'imu_node = segurito_production.imu_node:main',
            'drive_base = segurito_production.drive_base:main'
        ],
    },
)
