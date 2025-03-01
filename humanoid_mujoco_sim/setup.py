from setuptools import find_packages, setup

package_name = 'humanoid_mujoco_sim'

setup(
    name=package_name,
    version='0.0.0',
    #packages=find_packages(exclude=['test']),
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bth',
    maintainer_email='bth@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        "humanoid_sim=humanoid_mujoco_sim.humanoid_sim:main",
        "joy=humanoid_mujoco_sim.joy:main",
        "teleop=humanoid_mujoco_sim.teleop:main",     
        ],
    },
)
