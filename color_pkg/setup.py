import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'color_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        *( 
            [('share/' + package_name + '/launch', glob(os.path.join('launch', '*launch.[pxy][yma]*')))]
            if os.path.exists('launch') else []
        )
    ],
    install_requires=[
        'setuptools',
        'opencv-python',
        'cv_bridge',
    ],
    zip_safe=True,
    maintainer='dedperded',
    maintainer_email='hord.univer@gmail.com',
    description='A basic ROS 2 color detection package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'webcam_publisher_node = color_pkg.webcam_publisher_node:main',
            'color_detector_node = color_pkg.color_detector_node:main',
            'shape_detector_node = color_pkg.shape_detector_node:main',
        ],
    },
)
