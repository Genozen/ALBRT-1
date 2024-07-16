from setuptools import setup

package_name = 'mobile_robotics'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='genozen',
    maintainer_email='genozen@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'part1 = mobile_robotics.coding_ex1_part1:main',
            'cod_ex2 = mobile_robotics.coding_ex2:main',
        ],
    },
)
