import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'bringup_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'scripts'),
            glob('scripts/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='richard',
    maintainer_email='richard.solheim21@gmail.com',
    description='Top-level launch package — starts the full cube-pointing system',
    license='MIT',
    extras_require={'test': ['pytest']},
    entry_points={'console_scripts': [
        'publish_table=bringup_pkg.scripts.publish_table:main',
    ]},
)
