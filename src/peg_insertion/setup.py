import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'peg_insertion'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Tell the computer to pack our launch files!
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # Tell the computer to pack our Gazebo world files!
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
        # Tell the computer to pack our robot URDF blueprints!
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        # Tell the computer to pack our 3D meshes!
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*')),
        # Controller configuration
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='you@email.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'spiral_search = peg_insertion.spiral_search:main',
            'impedance_controller = peg_insertion.impedance_controller:main',
            'teleop_controller = peg_insertion.teleop_controller:main',
        ],
    },
)
