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
            "people_alert = segurito_production.people_alert:main",
            "person_stop  = segurito_production.person_stop:main",
            "map_mode_manager  = segurito_production.map_mode_manager:main",
            "video_recorder    = segurito_production.video_recorder:main",
        ],
    },
)
