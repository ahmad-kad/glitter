from setuptools import setup

package_name = 'glitter'

setup(
    name=package_name,
    version='1.0.0',
    packages=['glitter'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', [
            'config/l2_fusion.rviz',
            'config/l2_fusion_simple.rviz',
            'config/realsense_l2_combined.rviz'
        ]),
        ('share/' + package_name + '/config/tf', [
            'config/tf/static_transforms.launch.py',
            'config/tf/calibrate_transforms.py',
            'config/tf/README.md'
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='glitter',
    maintainer_email='user@glitter.local',
    description='LiDAR-Camera Fusion Package for RealSense and Unitree L2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fusion_node = glitter.fusion:main',
            'calibration_node = glitter.calibration:main',
        ],
    },
)