from setuptools import setup

package_name = 'py01_topic'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'demo01_talker_str_py   = py01_topic.demo01_talker_str_py:main',
            'demo01_listener_str_py = py01_topic.demo01_listener_str_py:main',
            'demo02_talker_stu   = py01_topic.demo02_talker_stu:main',
            'demo02_listener_stu = py01_topic.demo02_listener_stu:main'
        ],
    },
)
