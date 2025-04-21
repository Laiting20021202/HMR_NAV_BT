from setuptools import find_packages, setup

package_name = 'nav_planning_BT'

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
    maintainer='laiting',
    maintainer_email='laiting1202vip@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'basic_condition_testing_node = nav_planning_BT.basic_condition_testing_node:main',
            'nav_planning_BT_node = nav_planning_BT.nav_planning_BT_node:main',
            'nav_to_safe_point = nav_planning_BT.nav_to_safe_point:main',
            'situation_table_node = nav_planning_BT.situation_table_node:main',
            'nearest_wall_finder = nav_planning_BT.NearestWallFinder:main',
        ],
    },
)
