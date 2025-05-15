from setuptools import setup, find_packages
import os
import site
from glob import glob

package_name = 'my_vins_frontend'
# packages=find_packages(include=[package_name, package_name+'.*'])

# print('=================================',packages, submodules)
# print(f"Packages found: {', '.join(packages)}")
# print(f"{packages}")

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=[package_name, package_name + '.*']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + "/launch", glob("launch/*.py")),
        ('share/' + package_name + "/launch", glob("launch/*.xml")),
        ('share/' + package_name + "/launch", glob("launch/*.yaml"))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'spsg = ' + package_name + '.spsg:main'
        ],
    },
)
