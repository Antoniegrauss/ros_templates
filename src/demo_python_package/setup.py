from setuptools import find_packages, setup

package_name = 'demo_python_package'

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
    maintainer='developer',
    maintainer_email='antonie.grauss@alten.nl',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "number_counter = demo_python_package.number_counter:main",
            "simple_subscriber = demo_python_package.simple_subscriber:main",
            "add_two_ints_server = demo_python_package.add_two_ints_server:main",
            "add_two_ints_client = demo_python_package.add_two_ints_client:main",
            "hardware_status_publisher = demo_python_package.hardware_status_publisher:main"
        ],
    },
)
