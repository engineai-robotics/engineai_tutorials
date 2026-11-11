"""Declared ownership is checked against imports and plugin exports."""
import ast
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[2]
PACKAGES = ROOT/'src'


def test_platforms_and_infrastructure_do_not_import_application_layers():
    for package in ('core','perception','motion','hardware','sim','runtime'):
        for file in (PACKAGES/('pm01_navigation_'+package)).rglob('*.py'):
            if 'test' in file.parts: continue
            for node in ast.walk(ast.parse(file.read_text())):
                names = ([node.module or ''] if isinstance(node, ast.ImportFrom) else
                         [a.name for a in node.names] if isinstance(node, ast.Import) else [])
                for name in names:
                    assert not name.startswith(('pm01_navigation_client','pm01_navigation_console')), file
                    if package == 'runtime':
                        assert not name.startswith(('rclpy','rclcpp','nav2','gazebo','interface_protocol')), file


def test_packages_have_documentation_and_platform_dependencies_are_separate():
    for path in PACKAGES.glob('pm01_navigation_*'):
        assert (path/'README.md').is_file(), path
        assert (path/'test').is_dir(), path
        manifest = ET.parse(path/'package.xml').getroot()
        dependencies = {e.text for e in manifest if e.tag.endswith('depend')}
        if 'interface_protocol' in dependencies:
            assert path.name == 'pm01_navigation_hardware'
        if path.name == 'pm01_navigation_hardware': assert 'pm01_navigation_sim' not in dependencies
        if path.name == 'pm01_navigation_sim': assert 'pm01_navigation_hardware' not in dependencies
    assert not (PACKAGES/'pm01_navigation_adapters').exists()
    assert not (PACKAGES/'pm01_fastlio_nav').exists()


def test_plugin_lookup_names_are_exported_once():
    names=[]
    for file in PACKAGES.glob('pm01_navigation_*/*plugins.xml'):
        root=ET.parse(file).getroot()
        assert file.parent.name == 'pm01_navigation_nav2'
        names += [e.attrib.get('name', e.attrib['type']) for e in root.iter('class')]
    assert len(names) == len(set(names)) == 4
