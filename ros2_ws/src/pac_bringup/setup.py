from glob import glob
from setuptools import find_packages, setup
package_name='pac_bringup'
setup(name=package_name,version='0.1.0',packages=find_packages(),
 data_files=[('share/ament_index/resource_index/packages',['resource/'+package_name]),
             ('share/'+package_name,['package.xml']),
             ('share/'+package_name+'/launch',glob('launch/*.launch.py')),
             ('share/'+package_name+'/urdf',glob('urdf/*.xacro'))],
 install_requires=['setuptools'],zip_safe=True,maintainer='PAC 2026 Team',
 description='PAC 2026 launch orchestration.',license='Apache-2.0')
