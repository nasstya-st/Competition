from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'stepanova_anastasia_autorace_core'
package_name1 = 'robot_bringup'
package_name2 = 'robot_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name2, 'urdf'), glob(os.path.join('urdf', '*'))),
        (os.path.join('share', package_name1, 'worlds'), glob(os.path.join('worlds', '*.sdf'))),
        (os.path.join('share', package_name1, 'worlds'), glob(os.path.join('worlds', 'materials', '*'))),
        (os.path.join('share', package_name1, 'worlds'), glob(os.path.join('worlds', 'traffic_light', '*'))),
        (os.path.join('share', package_name1, 'config'), glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name1, 'config'), glob(os.path.join('config', '*.rviz'))),
        (os.path.join('share', package_name1, 'hooks'), glob(os.path.join('hooks', '*.in'))),
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
        'starting_alt = stepanova_anastasia_autorace_core.start_alt:main',
        'starting_alt2 = stepanova_anastasia_autorace_core.start_tonya:main',
        'dataset = stepanova_anastasia_autorace_core.dataset:main',

        ],
    },
)
