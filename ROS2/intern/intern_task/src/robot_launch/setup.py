from setuptools import find_packages, setup

package_name = 'robot_launch'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name,['package.xml']),
        ('share/' + package_name + '/launch', ['launch/runscript.launch.py'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='parichu',
    maintainer_email='orgaaume7741@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'delta_motor = robot_launch.delta_motor:main',
            'delta_odom = robot_launch.delta_odom:main',
        ],
    },
)
