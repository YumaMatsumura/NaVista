from setuptools import find_packages
from setuptools import setup

package_name = 'navista_cpu_monitor'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Yuma Matsumura',
    maintainer_email='yumapine@gmail.com',
    description='CPU Monitor for ROS 2',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['cpu_monitor = navista_cpu_monitor.cpu_monitor:main'],
    },
)
