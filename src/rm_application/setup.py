from setuptools import find_packages, setup

package_name = 'rm_application'

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
    maintainer='kie-dt46',
    maintainer_email='c1470759@outlook.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "init_robot_pose = rm_application.init_robot_pose:main",
            "get_robot_pose = rm_application.get_robot_pose:main",
            "nav_to_pose = rm_application.nav_to_pose:main",
            "waypoint_follower = rm_application.waypoint_follower:main",
        ],
    },
)
