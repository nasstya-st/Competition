from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'stepanova_anastasia_autorace_core'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*urdf.xacro'))),
        (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*.stl'))),
        (os.path.join('share', package_name, 'worlds'), glob(os.path.join('worlds', '*.sdf'))),
        (os.path.join('share', package_name, 'worlds'), glob(os.path.join('worlds', 'materials', '*'))),
        (os.path.join('share', package_name, 'worlds'), glob(os.path.join('worlds', 'traffic_light', '*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.rviz'))),
        (os.path.join('share', package_name, 'hooks'), glob(os.path.join('hooks', '*.in'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nastya',
    maintainer_email='nastasapervaa@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'starting = stepanova_anastasia_autorace_core.start:main',
        ],
    },
)
