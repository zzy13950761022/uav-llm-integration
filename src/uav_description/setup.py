from setuptools import find_packages, setup

package_name = 'uav_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/urdf', ['urdf/your_robot.urdf']),
        ('share/' + package_name + '/sdf', ['sdf/your_robot.sdf']),
        ('share/' + package_name + '/meshes', ['meshes/model.dae']),
        ('share/' + package_name + '/launch', ['launch/display.launch.py']),
        ('share/' + package_name + '/rviz', ['rviz/default.rviz']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='conan',
    maintainer_email='conanpodewitt@gmail.com',
    description='Package that represents the simulation description of the Pioneer UAV for use with RVIZ.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
