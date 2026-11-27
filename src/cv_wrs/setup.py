from setuptools import find_packages, setup

package_name = 'cv_wrs'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/models', [
            'models/valve_detector.pt',
            'models/gauge_regressor.pth',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Adithya',
    maintainer_email='adithyapothula123@gmail.com',
    description='Computer vision pipeline for WRS Aizu Spider disaster challenge',
    license='MIT',
    entry_points={
        'console_scripts': [
            'perception_node = cv_wrs.perception_node:main',
        ],
    },
)
