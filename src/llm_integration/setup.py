from setuptools import setup
from glob import glob
import os

package_name = 'llm_integration'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        # Install launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # Install package.xml
        (os.path.join('share', package_name), ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Conan Dewitt',
    maintainer_email='22877792@student.uwa.edu.au',
    description='Integration of LLM into UAV simulation for motion control.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'llm_node = llm_integration.llm_node:main',
            'text_in_node = llm_integration.text_in_node:main',
        ],
    },
)
