import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'algae_dt'


def _data_files():
    """Install the ament marker, manifest, and every asset dir preserving structure."""
    files = [
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ]
    for sub in ('launch', 'config', 'worlds', 'maps'):
        for path in glob(os.path.join(sub, '**', '*'), recursive=True):
            if os.path.isfile(path):
                files.append((os.path.join('share', package_name, os.path.dirname(path)), [path]))
    return files


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=_data_files(),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='algae-dt team',
    maintainer_email='team36@student.tue.nl',
    description='TurtleBot3 Burger digital-twin DT layer (TU/e 2IRR10) on the default turtlebot3 stack.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'twin_mediator = algae_dt.twin_mediator:main',
            'sync_supervisor = algae_dt.sync_supervisor:main',
            'mission_runner = algae_dt.mission_runner:main',
            'operator_gui = algae_dt.operator_gui:main',
        ],
    },
)
