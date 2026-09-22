from setuptools import find_packages, setup

package_name = 'corridor_sim'

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
    maintainer='singh_nav',
    maintainer_email='singh_nav@todo.todo',
    description='Shifting Wall & Slipping Base Challenge — corridor world and panel mover',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wall_panel_mover = corridor_sim.wall_panel_mover:main',
        ],
    },
)
