from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'kamera_tf2_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch','*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name), glob('config/*config.rviz'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nikolaakrap',
    maintainer_email='nikola.akrap02@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'kamera_tf2 = kamera_tf2_py.kamera_tf2:main',
            'static_marker = kamera_tf2_py.static_marker:main',
            'move_robot = kamera_tf2_py.move_robot:main',
            'gather_info = kamera_tf2_py.gather_info:main'
        ],
    },
)
