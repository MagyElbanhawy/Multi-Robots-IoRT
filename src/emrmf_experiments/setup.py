from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'emrmf_experiments'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test', 'scripts']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='EMRMF Multi-Robot SLAM Experiment Logger',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'network_proxy_node = emrmf_experiments.network_proxy_node:main',
            'experiment_logger_node = emrmf_experiments.experiment_logger_node:main',
            'orchestrator = scripts.orchestrator:main',
            'report_generator = scripts.report_generator:generate_reports'
        ],
    },
)
