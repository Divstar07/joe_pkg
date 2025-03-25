from setuptools import find_packages, setup

package_name = 'joe_pkg'

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
    maintainer='divine',
    maintainer_email='dndukac2007@gmail.com',
    description='Functionality for JOE: MREN 203',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'key_twist_publisher = joe_pkg.twist_pub:main'
        ],
    },
)
