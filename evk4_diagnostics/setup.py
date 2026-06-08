from setuptools import find_packages, setup

package_name = 'evk4_diagnostics'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Modestas',
    maintainer_email='modesuka@gmail.com',
    description='Stream-health watchdog for the Prophesee EVK4, '
                'publishing diagnostic_msgs on /diagnostics.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'camera_monitor = evk4_diagnostics.camera_monitor:main',
        ],
    },
)
