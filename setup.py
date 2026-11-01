from setuptools import find_packages, setup

package_name = 'ax12_control'

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
    maintainer='jun',
    maintainer_email='jun@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
       entry_points={
        'console_scripts': [
            'monitor_plot = ax12_control.monitor_plot:main',
            'key_teleop = ax12_control.key_teleop:main',
            'arm_controller = ax12_control.arm_controller:main',
        ],
    },
)
