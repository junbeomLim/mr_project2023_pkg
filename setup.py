from setuptools import find_packages
from setuptools import setup

package_name = 'mr_project2023_pkg'

setup(
    name=package_name,
    version='0.1.3',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='limjunbeom',
    maintainer_email='limjunbeom@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'learn_node = mr_project2023_pkg.learning:main',
            'robotarm = mr_project2023_pkg.robotarm:main'
        ],
    },
)
