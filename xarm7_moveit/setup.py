from setuptools import find_packages, setup
from glob import glob

package_name = 'xarm7_moveit'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/meshes', glob('meshes/*.stl')),
        ('share/' + package_name + '/launch', glob('launch/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='e25vanac',
    maintainer_email='e25vanac@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'xarm7_move = xarm7_moveit.xarm7_move:main',
            'xarm7_circle = xarm7_moveit.xarm7_circle:main',
            'xarm7_gripper = xarm7_moveit.xarm7_gripper:main',
            'xarm7_dumb_pick_place = xarm7_moveit.xarm7_dumb_pick_place:main',
            'xarm7_linear_motor = xarm7_moveit.xarm7_linear_motor:main',
            'xarm7_square = xarm7_moveit.xarm7_square:main',
            'xarm7_collision = xarm7_moveit.xarm7_collision:main',
            'xarm7_dumb_pick_pour = xarm7_moveit.xarm7_dumb_pick_pour:main',
            'xarm7_servo_gripper = xarm7_moveit.xarm7_servo_gripper:main',
            'xarm7_servo_linear_track = xarm7_moveit.xarm7_servo_linear_track:main',
            'xarm7_bol = xarm7_moveit.xarm7_bol:main',
            'xarm7_demo = xarm7_moveit.xarm7_demo:main',
            'xarm7_cam_pick_pour = xarm7_moveit.xarm7_cam_pick_pour:main',
            'convert_ros_cv = xarm7_moveit.convert_ros_cv:main'
        ],
    },
)