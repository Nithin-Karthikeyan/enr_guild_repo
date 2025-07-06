from setuptools import find_packages, setup
from glob import glob

package_name = 'enr_guild_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/urdf', glob('urdf/*.urdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nithin',
    maintainer_email='nithinkarthikeyan01@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'talker = enr_guild_pkg.talker:main',
        'relay = enr_guild_pkg.relay:main',
        'circle = enr_guild_pkg.circle:main',
        'webcam = enr_guild_pkg.webcam:main',
        'bottle_rf = enr_guild_pkg.bottle_rf:main',
        'keyboard_control = enr_guild_pkg.keyboard_control:main',
        ],
    },
)
