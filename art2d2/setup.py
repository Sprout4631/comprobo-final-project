from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'art2d2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.py')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='alana',
    maintainer_email='sprout4631@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'drawing_node = art2d2.drawing_node:main',
            'localization_node = art2d2.localization_node:main'
        ],
    },
)
