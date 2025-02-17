from setuptools import find_packages, setup

package_name = 'llm_integration'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'rclpy', 'requests', 'opencv-python', 'cv_bridge'],
    zip_safe=True,
    maintainer='Conan Dewitt',
    maintainer_email='22877792@student.uwa.edu.au',
    description='Integration of LLM into UAV simulation for motion control.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'llm_node = llm_integration.llm_node:main'
        ],
    },
)
