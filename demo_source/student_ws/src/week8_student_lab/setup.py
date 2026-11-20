from setuptools import setup
package_name='week8_student_lab'
setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',['resource/'+package_name]),
        ('share/'+package_name,['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Course Staff',
    maintainer_email='course@example.invalid',
    description='Week 8 from-zero RViz lab: A*, virtual navigation, mission and exploration',
    license='Apache-2.0',
    entry_points={'console_scripts':[
        'starter = week8_student_lab.starter_map:main',
        'demo = week8_student_lab.ros_demo:main',
    ]},
)
