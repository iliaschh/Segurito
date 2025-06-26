from setuptools import find_packages, setup

package_name = 'segurito_exploration_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            [f'resource/{package_name}']),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/web_backend.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='neverdiedooms',
    maintainer_email='ismael-fernandez@outlook.es',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'exploration_service = segurito_exploration_control.exploration_service:main',
            'people_detector = segurito_exploration_control.people_detector:main',
            'map_files_server = segurito_exploration_control.map_files_server:main',
            "video_files_server = segurito_exploration_control.video_files_server:main",
        ],
    },
)
