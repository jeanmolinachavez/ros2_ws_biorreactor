from setuptools import setup

package_name = 'nodo_central_ros'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='TuNombre',
    maintainer_email='tu@email.com',
    description='Nodo central que recibe datos del ESP32',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'nodo_central = nodo_central_ros.nodo_central:main',
            'nodo_exterior = nodo_central_ros.nodo_exterior:main',
            'nodo_arduino_serial = nodo_central_ros.nodo_arduino_serial:main',
        ],
    },
)
