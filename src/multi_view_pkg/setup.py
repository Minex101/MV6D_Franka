from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'multi_view_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Affan Mohammed',
    maintainer_email='affanmohd8khan@gmail.com/el22maak@leeds.ac.uk',
    description='ROS 2 package for Multi-View Object Pose Fusion using Nvidia DOPE, TRAC-IK and Markley\'s method.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fusion4 = multi_view_pkg.fusion_node_4:main',
            'movement4 = multi_view_pkg.movement_node_4:main',
            'vision4 = multi_view_pkg.vision_node_4:main',
        ],
    },
)