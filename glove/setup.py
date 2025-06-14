from setuptools import find_packages, setup

package_name = 'glove'

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
    maintainer='keshaw',
    maintainer_email='lucky7chess@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'read_and_send = glove.read_and_send:main',
            'read_and_send_zmq = glove.read_and_send_zmq:main'
        ],
    },
)
