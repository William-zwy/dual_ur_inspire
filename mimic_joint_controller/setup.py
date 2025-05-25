from setuptools import find_packages, setup

package_name = 'mimic_joint_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/mimic.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zwy',
    maintainer_email='20213232057@m.scnu.edu.cn',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mimic_node = mimic_joint_controller.mimic_node:main'
        ],
    },
)
