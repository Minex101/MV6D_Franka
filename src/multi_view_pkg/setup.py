from setuptools import find_packages, setup

package_name = 'multi_view_pkg'

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
    maintainer='Affan Mohammed',
    maintainer_email='el22maak@leeds.ac.uk',
    description='ROS 2 package for Multi-View Object Pose Fusion using Nvidia DOPE, TRAC-IK and Markley\'s method.',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'vision = multi_view_pkg.vision_node:main',
            'movement = multi_view_pkg.movement_node:main',
            'fusion = multi_view_pkg.fusion_node:main',
            'vision2 = multi_view_pkg.vision_node_2:main',
            'movement2 = multi_view_pkg.movement_node_2:main',
            'movement3 = multi_view_pkg.movement_node_3:main',
            'fusion2 = multi_view_pkg.fusion_node_2:main',
        ],
    },
)
