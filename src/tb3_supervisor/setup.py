from setuptools import setup

package_name = 'tb3_supervisor'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name, f'{package_name}.utils'],
    package_dir={'': '.'},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/sim_world.launch.py',
            'launch/nav2_supervised.launch.py',
            'launch/bringup_all.launch.py',
            'launch/slam_and_map_save.launch.py',
        ]),
        ('share/' + package_name + '/config', [
            'config/nav2_params.yaml',
        ]),
        # If you add a map later:
        # ('share/' + package_name + '/maps', ['maps/turtlebot3_world.yaml', 'maps/turtlebot3_world.pgm']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Shrikrishna',
    author_email='shrikrishnarb@gmail.com',
    description='Nav2 + TB3 Gazebo with safety supervisor',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'supervisor_node = tb3_supervisor.supervisor_node:main',
        ],
    },
)