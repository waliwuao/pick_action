from glob import glob
from setuptools import find_packages, setup

package_name = 'pick_action'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='charlie',
    maintainer_email='charlie@example.com',
    description='LiDAR-based autonomous pick sequence action.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pick_action_server_node = pick_action.pick_action_server:main',
            'recognition_node = pick_action.recognition_node:main',
            'synthetic_scan_node = pick_action.synthetic_scan_node:main',
            'trigger_pick = pick_action.trigger_pick:main',
        ],
    },
)
