from setuptools import setup, find_packages
from glob import glob
setup(name='pm01_navigation_client', version='1.0.0',
      packages=find_packages(exclude=['test']),
      data_files=[('share/ament_index/resource_index/packages', ['resource/pm01_navigation_client']),
                  ('share/pm01_navigation_client', ['package.xml']),
                  ('share/pm01_navigation_client/examples', glob('examples/*.py'))],
      install_requires=['setuptools'], tests_require=['pytest'], zip_safe=True,
      maintainer='PM01 maintainers', maintainer_email='info@engineai.com.cn',
      description='Standard ROS2 navigation client and bounded diagnostics', license='Apache-2.0',
      entry_points={'console_scripts': ['pm01_nav = pm01_navigation_client.cli:main',
                                       'pm01_sim_keyboard = pm01_navigation_client.keyboard:main',
                                       'pm01_clock_check = pm01_navigation_runtime.system.clock_check:main',
                                       'pm01_nav_debug = pm01_navigation_client.debug:main']})
