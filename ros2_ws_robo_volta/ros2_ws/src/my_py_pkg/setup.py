from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'my_py_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robot',
    maintainer_email='robot@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'talker = my_py_pkg.talker:main',
        'listener = my_py_pkg.listener:main',
        'turtle_controller = my_py_pkg.turtle_controller:main',
        'turtle_pose_reader = my_py_pkg.turtle_pose_reader:main',
        'controller2 = my_py_pkg.turtle_controller2:main',
        ],
    },
)
