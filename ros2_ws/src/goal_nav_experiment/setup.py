from setuptools import setup

package_name = 'goal_nav_experiment'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/worlds', ['worlds/arena.sdf']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mohamed Tawakol',
    maintainer_email='7952397@studenti.unige.it',
    description='Go-to-goal navigation experiment simulator for RT2 final assignment',
    license='MIT',
    entry_points={
        'console_scripts': [
            'experiment_node = goal_nav_experiment.experiment_node:main',
            'gazebo_viz = goal_nav_experiment.gazebo_viz:main',
        ],
    },
)
