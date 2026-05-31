from setuptools import find_packages, setup

package_name = 'emrmf_core'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='EMRMF core package for Trust Factor and Global Fusion',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'trust_factor_node = emrmf_core.trust_factor_node:main',
            'global_map_fusion_node = emrmf_core.global_map_fusion_node:main',
            'dynamic_task_allocation_node = emrmf_core.dynamic_task_allocation_node:main',
            'experiment_logger = emrmf_core.experiment_logger:main'
        ],
    },
)
