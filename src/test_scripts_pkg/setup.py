from setuptools import find_packages, setup

package_name = 'test_scripts_pkg'

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
    maintainer='affan',
    maintainer_email='affanmohd8khan@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'poseacctest = test_scripts_pkg.PoseAccuracy:main',
            'oriacctest = test_scripts_pkg.OrientationAccuracy:main'
        ],
    },
)
