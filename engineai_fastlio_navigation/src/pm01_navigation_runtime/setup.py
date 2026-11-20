from setuptools import setup, find_packages
setup(name='pm01_navigation_runtime', version='1.0.0', packages=find_packages(exclude=['test']),
      data_files=[('share/ament_index/resource_index/packages', ['resource/pm01_navigation_runtime']),
                  ('share/pm01_navigation_runtime', ['package.xml', 'README.md'])],
      package_data={'pm01_navigation_runtime': ['build/profiles.json', 'release/sources.json']},
      install_requires=['setuptools'], tests_require=['pytest'], zip_safe=True,
      maintainer='PM01 maintainers', maintainer_email='info@engineai.com.cn',
      description='配置、进程、构建与部署基础能力；导入不启动进程，不依赖 ROS。', license='Apache-2.0',
      entry_points={'console_scripts': []})
