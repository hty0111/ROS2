import os
from glob import glob

from setuptools import setup

package_name = 'learning_tf2_python'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hty',
    maintainer_email='hty@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tf2_broadcaster = learning_tf2_python.frame_broadcaster:main',
            'tf2_listener = learning_tf2_python.frame_listener:main',
            'fixed_tf2_broadcaster = learning_tf2_python.fixed_frame_broadcaster:main',
        ],
    },
)
