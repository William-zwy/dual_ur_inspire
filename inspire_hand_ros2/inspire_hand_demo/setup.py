from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'inspire_hand_demo'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # Include any other launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='dobot',
    maintainer_email='dobot@todo.todo',
    description='Inspire Hand ROS2 control package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'inspire_hand_bringup = inspire_hand_demo.inspire_hand_bringup:main',
            'inspire_hand_publisher = inspire_hand_demo.inspire_pub:main',
            'inspire_hand_subscriber = inspire_hand_demo.inspire_sub:main',
            'inspire_client = inspire_hand_demo.insipre_client:main',
            # Legacy aliases for backward compatibility
            'talker = inspire_hand_demo.inspire_pub:main',
            'client = inspire_hand_demo.inspire_sub:main',
        ],
    },
)
