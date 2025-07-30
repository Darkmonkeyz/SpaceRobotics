from glob import glob

from setuptools import find_packages, setup

package_name = 'particle_filter_localisation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*launch.[pxy][yma]*')),
        ('share/' + package_name + '/config', glob('config/*')),
        ('share/' + package_name + '/world_data', glob('world_data/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Graeme Best',
    maintainer_email='graeme.best@uts.edu.au',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'particle_filter_localisation = particle_filter_localisation.particle_filter_localisation:main',
            'visual_terrain_sensor = particle_filter_localisation.visual_terrain_sensor:main',
            'mock_magnetometer = particle_filter_localisation.mock_magnetometer:main'
        ],
    },
)
