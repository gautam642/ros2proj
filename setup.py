from setuptools import find_packages, setup

package_name = 'serial_adc_pid'

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
    maintainer='gautam',
    maintainer_email='gautam@todo.todo',
    description='Serial reader and PID controller',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'serial_adc_publisher = serial_adc_pid.serial_adc_publisher:main',
            'pid_controller = serial_adc_pid.pid_controller:main',
            'motor_sim = serial_adc_pid.motor_sim:main',
        ]
    },
)
