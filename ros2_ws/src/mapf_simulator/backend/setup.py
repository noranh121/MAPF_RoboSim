from setuptools import setup
import os

from setuptools import find_packages
from glob import glob

package_name = 'backend'

setup(
    name=package_name,
    version='0.0.0',
    # packages=[package_name],
    packages=find_packages(include=[package_name, 'algorithms', 'algorithms.*']),

    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']), ('share/' + package_name + '/params', glob('params/*')), ('share/' + package_name + '/urdf', glob('urdf/*')), ('share/' + package_name + '/models/turtlebot3_burger', glob('models/turtlebot3_burger/*')), ('share/' + package_name + '/models/turtlebot3_waffle', glob('models/turtlebot3_waffle/*')), ('share/' + package_name + '/worlds', glob('worlds/*')),('share/' + package_name + '/algorithms', glob('algorithms/*.py')),('share/' + package_name + '/algorithms/pycam', glob('algorithms/pycam/*.py')), (os.path.join('share', package_name), glob('launch/*.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['backend_engine.py = backend.backend_engine:main'],
    },
)