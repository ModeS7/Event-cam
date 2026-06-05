from setuptools import find_packages, setup

package_name = 'evk4_examples'

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
    description='Example Python subscribers for the Prophesee EVK4 '
                'event camera topics published by evk4_bringup.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'event_rate = evk4_examples.event_rate_node:main',
        ],
    },
)
