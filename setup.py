from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'towtruck_testing'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # install .msg and .srv files
        (os.path.join('share', package_name, 'msg'), glob('msg/*.msg')),
        (os.path.join('share', package_name, 'srv'), glob('srv/*.srv')),
        (os.path.join('share', package_name, package_name), glob(f'{package_name}/*.py')),
        (os.path.join('share', package_name), ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nontanan',
    maintainer_email='nontanan@gensurv.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'towtruck_get_path = towtruck_testing.towtruck_get_behavior_path:main',
            'towtruck_mission_func = towtruck_testing.towtruck_mission_func:main',
            'towtruck_publish_marker_points = towtruck_testing.towtruck_publish_marker_points:main',
            'towtruck_record_points = towtruck_testing.towtruck_record_points.py',
            'test_towtruck_ui = towtruck_testing.test_towtruck_ui:main',
            'test_towtruck_latency = towtruck_testing.test_towtruck_latency:main',
            'unit_edges = towtruck_testing.unit_edges:main',
        ],
    },
)
