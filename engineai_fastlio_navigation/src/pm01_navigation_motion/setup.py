from setuptools import setup, find_packages
setup(name='pm01_navigation_motion', version='1.0.0', packages=find_packages(exclude=['test']),
      data_files=[('share/ament_index/resource_index/packages', ['resource/pm01_navigation_motion']),
                  ('share/pm01_navigation_motion', ['package.xml', 'README.md'])],
      install_requires=['setuptools'], tests_require=['pytest'], zip_safe=True,
      maintainer='PM01 maintainers', maintainer_email='info@engineai.com.cn',
      description='运动输出接通、释放和 worker 生命周期管理，默认关闭输出。', license='Apache-2.0',
      entry_points={'console_scripts': ['motion_manager.py = pm01_navigation_motion.manager:main']})
