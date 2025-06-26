from setuptools import find_packages, setup

package_name = 'rtde_pkg'

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
    maintainer='root',
    maintainer_email='panjisongcumt@126.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "test_rtde_node = rtde_pkg.test_rtde_node:main",
            "receiver_node = rtde_pkg.rtde_dual_receiver:main",
            "left_ctrl_node = rtde_pkg.rtde_left_controller:main",
            "right_ctrl_node = rtde_pkg.rtde_right_controller:main"
        ],
    },
)
