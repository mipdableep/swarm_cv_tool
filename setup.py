from setuptools import setup

package_name = 'swarm_env_detector'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ido_Talmor',
    maintainer_email='ifw.talmor@gmail.com',
    description='Localization_and_peer_detection_module',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'location_peer_detection = swarm_env_detector.location_peer_detection:main'
        ],
    },
)
