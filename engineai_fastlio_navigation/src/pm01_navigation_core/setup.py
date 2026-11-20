from setuptools import Extension, setup, find_packages
setup(name='pm01_navigation_core', version='1.0.0', packages=find_packages(exclude=['test']),
      package_data={'pm01_navigation_core.mapping': ['ray_samples.cuh']},
      data_files=[('share/ament_index/resource_index/packages', ['resource/pm01_navigation_core']),
                  ('share/pm01_navigation_core', ['package.xml', 'README.md'])],
      ext_modules=[Extension('_pm01_evidence', ['src/evidence_projection.cpp'],
                             language='c++', extra_compile_args=['-std=c++17', '-O3', '-ffp-contract=off'])],
      install_requires=['setuptools'], tests_require=['pytest'], zip_safe=False,
      maintainer='PM01 maintainers', maintainer_email='info@engineai.com.cn',
      description='自动地面、实时占用地图与定位结果接入。', license='Apache-2.0',
      entry_points={'console_scripts': ['automatic_ground.py = pm01_navigation_core.nodes.automatic_ground:main', 'navigation_obstacle_cloud.py = pm01_navigation_core.nodes.navigation_obstacle_cloud:main', 'online_navigation_map.py = pm01_navigation_core.nodes.online_navigation_map:main', 'map_matching_adapter.py = pm01_navigation_core.nodes.map_matching_adapter:main']})
