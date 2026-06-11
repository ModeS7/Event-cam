from setuptools import find_packages, setup

package_name = 'evk4_calibration'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/calibrate.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Modestas',
    maintainer_email='modesuka@gmail.com',
    description='Guided intrinsic calibration for the Prophesee EVK4.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'calibrate = evk4_calibration.calibrate:main',
        ],
    },
)
