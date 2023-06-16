from setuptools import setup

package_name = 'ugv3'
data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name + '/launch', ['launch/robot_launch.py']))
data_files.append(('share/' + package_name + '/worlds', ['worlds/ugvWorld.wbt']))
data_files.append(('share/' + package_name + '/resource', ['resource/my_robot.urdf']))
data_files.append(('share/' + package_name + '/resource/model3D', ['resource/model3D/camera.obj']))
data_files.append(('share/' + package_name, ['package.xml']))
setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sergio',
    maintainer_email='sergio@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_robot_driver = ugv3.my_robot_driver:main',
            'auto_alg = ugv3.auto_alg:main'
        ],
    },
)
