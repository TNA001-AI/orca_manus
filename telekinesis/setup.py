from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'telekinesis'

def get_mesh_files():
    mesh_files = []
    # Add URDF files
    mesh_files.append((os.path.join('share', package_name, 'telekinesis', 'orca_hand'), 
                      glob('telekinesis/orca_hand/*.urdf')))
    
    # Add mesh files for both left and right hands
    for hand in ['left', 'right']:
        for mesh_type in ['visual', 'collision']:
            mesh_path = os.path.join('telekinesis', 'orca_hand', 'mesh', hand, mesh_type)
            if os.path.exists(mesh_path):
                mesh_files.append((os.path.join('share', package_name, 'telekinesis', 'orca_hand', 'mesh', hand, mesh_type),
                                 glob(os.path.join(mesh_path, '*'))))
    return mesh_files

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ] + get_mesh_files(),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='keshaw',
    maintainer_email='lucky7chess@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'leap_ik = telekinesis.leap_ik:main',
            'orca_ik = telekinesis.orca_ik:main'
        ],
    },
)
