import os
from setuptools import find_packages, setup

package_name = 'uav_description'

# Function to gather all files in a directory
def package_files(directory):
    paths = []
    for (path, directories, filenames) in os.walk(directory):
        for filename in filenames:
            paths.append(os.path.join(path, filename))
    return paths

# Gather all files from the meshes directory
mesh_files = package_files('meshes')

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/urdf', ['urdf/pioneer.urdf']),
        ('share/' + package_name + '/sdf', ['sdf/world.sdf']),
        ('share/' + package_name + '/launch', ['launch/simulation.launch.py']),
        ('share/' + package_name + '/rviz', ['rviz/default.rviz']),
    ] + [(os.path.join('share', package_name, os.path.dirname(f)), [f]) for f in mesh_files],
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
