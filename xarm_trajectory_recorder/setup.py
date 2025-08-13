from setuptools import find_packages, setup

package_name = 'xarm_trajectory_recorder'

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
    maintainer='e25vanac',
    maintainer_email='eleonore.vanacker@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'trajectory_recorder=xarm_trajectory_recorder.trajectory_recorder:main',
            'trajectory_planner=xarm_trajectory_recorder.trajectory_planner:main'
        ],
    },
)
