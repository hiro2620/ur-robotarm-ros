from setuptools import find_packages, setup

package_name = 'ur5_ft_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['ur5_ft_control/launch/ur5_ft_control.launch.py']),
        ('share/' + package_name + '/launch', ['ur5_ft_control/launch/ur5_ft_moveit_gazebo.launch.py']),
        ('share/' + package_name + '/urdf', ['ur5_ft_control/urdf/ur5_ft300.urdf.xacro']),
        ('share/' + package_name + '/config', ['ur5_ft_control/srdf/ur5_ft300.srdf.xacro'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='ihiro2620@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ur5_ft_controller = ur5_ft_control.ur5_ft_controller:main',
        ],
    },
)
