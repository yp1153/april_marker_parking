from setuptools import find_packages, setup

package_name = 'april_parking'

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
    maintainer='cowin-rsp',
    maintainer_email='cowin-rsp@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'img2april_data = april_parking.img2april_data:main',
            'image_pub = april_parking.image_pub:main'
        ],
    },
)
