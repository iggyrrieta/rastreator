from setuptools import setup
import os
import glob

package_name = 'rastreator_simulation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'param'),glob.glob('param/*.yaml')),
        (os.path.join('share', package_name, 'launch'),glob.glob('launch/*.launch.py')),
        (os.path.join('lib', package_name, 'utils'),glob.glob('utils/*.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='iggyrrieta',
    maintainer_email='lorente.inaki@gmail.com',
    description='rastreator_simulation: Simulation package',
    license='Apache 2.0',
    entry_points={
        'console_scripts': [
            'ekf = rastreator_simulation.ekf_simulation:main',
            'constant_cmd = rastreator_simulation.constant_cmd:main',
        ],
    },
)
