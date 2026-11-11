import os
from pathlib import Path

CONFIG = Path(__file__).resolve().parents[2] / "config/console"
if not (CONFIG / "console.example.yaml").is_file():
    raise RuntimeError("Missing workspace config/console; use the complete project source tree")
from setuptools import setup

setup(name='pm01_navigation_console', version='1.0.0',
      packages=['pm01_navigation_console'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/pm01_navigation_console']),
                  ('share/pm01_navigation_console', ['package.xml']),
                  ('share/pm01_navigation_console/config', [os.path.relpath(CONFIG / 'console.example.yaml', Path(__file__).resolve().parent)])],
      install_requires=['setuptools'], tests_require=['pytest'], zip_safe=True,
      maintainer='PM01 maintainers', maintainer_email='info@engineai.com.cn',
      description='Native Ubuntu PM01 console', license='Apache-2.0',
      entry_points={'console_scripts': ['pm01_console = pm01_navigation_console.app:main']})
