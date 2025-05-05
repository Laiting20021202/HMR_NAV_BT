from setuptools import find_packages, setup

package_name = 'lidar_realsense_combination'

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
        'console_scripts': [ 'realsense_pointcloud_pub = lidar_realsense_combination.realsense_pointcloud_pub:main',
        ],
    },
)
