from setuptools import find_packages, setup

package_name = 'cobot1'

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
    maintainer='deepday',
    maintainer_email='deepday@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'mini_jog = cobot1.mini_jog:main',
            'dot_drawer_action = cobot1.dot_drawer_action:main',
            'dot_drawer_action_dev = cobot1.dot_drawer_action_dev:main',
            'dot_dummy_client = cobot1.dot_dummy_client:main',
            'test_gipper = cobot1.test_gipper:main',
            'check_robot_state_recovery = cobot1.check_robot_state_recovery:main',
        ],
    },
)
