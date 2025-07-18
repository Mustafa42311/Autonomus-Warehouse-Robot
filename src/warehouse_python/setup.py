from setuptools import find_packages, setup

package_name = 'warehouse_python'

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
    maintainer='mustafa',
    maintainer_email='mustafa@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
             'simple_tf_kinematics = warehouse_python.simple_tf_kinematics:main',
             'simple_lifecycle_node = warehouse_python.simple_lifecycle_node:main',
        ],
    },
)
