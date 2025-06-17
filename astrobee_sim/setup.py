from setuptools import find_packages, setup
import os
from glob import glob


package_name = 'astrobee_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/ament_index/resource_index/packages',
            ['resource/' + 'visualize.rviz']),
        ('share/ament_index/resource_index/packages',
            ['resource/' + 'astrobee_sim_bridge.yaml']),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'resource'), glob('resource/*')),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tom',
    maintainer_email='tommaso.faraci3@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rotate_robot = astrobee_sim.rotate_robot:main',
            'execute_trajectory = astrobee_sim.execute_trajectory:main',
        ],
    },
)
