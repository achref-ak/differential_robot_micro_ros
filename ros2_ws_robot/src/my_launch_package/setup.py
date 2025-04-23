from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'my_launch_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),  # This is the key change
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Add your executables here if any
        ],
    },
)
