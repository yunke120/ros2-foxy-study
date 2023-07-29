from setuptools import setup
from glob import glob
package_name = 'py01_launch'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, glob("launch/*_launch.py")),
        ('share/' + package_name, glob("launch/*_launch.xml")),
        ('share/' + package_name, glob("launch/*_launch.yaml")),
        # ('share/' + package_name, ['launch/py01_helloworld_launch.py']),
        # ('share/' + package_name, ['launch/xml01_helloworld_launch.xml']),
        # ('share/' + package_name, ['launch/yaml01_helloworld_launch.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros',
    maintainer_email='fj_18860352816@163.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
