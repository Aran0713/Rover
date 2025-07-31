from setuptools import find_packages, setup

package_name = 'go_to_service'

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
    maintainer='aran',
    maintainer_email='aran0713@hotmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'drive_distance_server = go_to_service.drive_distance_server:main',
            'go_to_service = go_to_service.go_to_service:main',
        ],
    },
)
