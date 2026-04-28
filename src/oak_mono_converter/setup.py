from setuptools import setup

package_name = 'oak_mono_converter'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Converts OAK-D simulated RGB mono cameras to grayscale mono8',
    license='MIT',
    entry_points={
        'console_scripts': [
            'mono_converter = oak_mono_converter.mono_converter_node:main',
        ],
    },
)
