"""One build selection for source, Docker and incremental workspaces."""
import argparse
from importlib import resources
import json
from pathlib import Path
import shutil
import time


def manifest():
    return json.loads(resources.files(__package__).joinpath('profiles.json').read_text())


def packages(profile, stage='all'):
    names = manifest()['profiles'][profile]
    if stage == 'driver': return ['livox_ros_driver2']
    if stage == 'lio': return ['fast_lio']
    if stage == 'navigation': return [n for n in names if n not in ('livox_ros_driver2', 'fast_lio')]
    return list(names)


def source(package):
    return manifest()['sources'].get(package, 'src/'+package)


def prepare_layout(workspace):
    """Retire only generated package outputs from the old mixed-package layout."""
    workspace = Path(workspace)
    marker = workspace/'build/.pm01-layout.json'
    if marker.exists() and json.loads(marker.read_text()).get('schema') == 2:
        return
    own = {p for values in manifest()['profiles'].values() for p in values if p.startswith('pm01_')}
    own |= {'pm01_navigation_adapters', 'pm01_fastlio_nav', 'lidar_localization_ros2', 'ndt_omp'}
    backup = workspace/'build'/('.retired-layout-'+str(time.time_ns()))
    for output in ('install', 'build'):
        for package in own:
            path = workspace/output/package
            if path.exists() or path.is_symlink():
                target = backup/output/package
                target.parent.mkdir(parents=True, exist_ok=True)
                shutil.move(str(path), str(target))
    marker.parent.mkdir(parents=True, exist_ok=True)
    marker.write_text(json.dumps({'schema': 2})+'\n')


def main(workspace):
    parser = argparse.ArgumentParser()
    parser.add_argument('profile', choices=manifest()['profiles'])
    parser.add_argument('--stage', default='all', choices=['all','driver','lio','navigation'])
    parser.add_argument('--prepare-layout', action='store_true')
    args = parser.parse_args()
    if args.prepare_layout:
        prepare_layout(workspace)
        prepare_source_layout(workspace, args.profile, args.stage)
        prepare_config_layout(workspace, args.profile, args.stage)
        prepare_pruned_layout(workspace, args.profile, args.stage)
    print('\n'.join(source(name) for name in packages(args.profile, args.stage)))


def prepare_source_layout(workspace, profile, stage='all'):
    """Retire CMake paths and package indexes only when their platform is built."""
    if profile not in ('sim', 'hardware') or stage not in ('all', 'navigation'):
        return
    workspace = Path(workspace)
    marker = workspace/'install/.pm01-source-layout.json'
    migrated = set(json.loads(marker.read_text()).get('vendored', [])) if marker.exists() else set()
    affected = {'slam_toolbox', 'karto_sdk'}
    if profile == 'sim':
        affected |= {'simulator', 'rl_walking'}
    pending = affected - migrated
    if not pending:
        return
    backup = workspace/'build'/('.retired-source-layout-'+str(time.time_ns()))
    for output in ('build', 'install'):
        for package in sorted(pending):
            path = workspace/output/package
            if path.exists() or path.is_symlink():
                target = backup/output/package
                target.parent.mkdir(parents=True, exist_ok=True)
                shutil.move(str(path), str(target))
    marker.parent.mkdir(parents=True, exist_ok=True)
    marker.write_text(json.dumps({'vendored': sorted(migrated | affected)})+'\n')


def prepare_config_layout(workspace, profile, stage='all'):
    """Rebuild resource packages once after their defaults move to config/."""
    if stage not in ('all', 'navigation'):
        return
    affected = set(packages(profile)) & {'pm01_navigation_bringup', 'pm01_navigation_console'}
    workspace = Path(workspace)
    marker = workspace/'build/.pm01-config-layout.json'
    migrated = set(json.loads(marker.read_text()).get('root_config', [])) if marker.exists() else set()
    pending = affected - migrated
    if not pending:
        return
    backup = workspace/'build'/('.retired-config-layout-'+str(time.time_ns()))
    for output in ('build', 'install'):
        for package in sorted(pending):
            path = workspace/output/package
            if path.exists() or path.is_symlink():
                target = backup/output/package
                target.parent.mkdir(parents=True, exist_ok=True)
                shutil.move(str(path), str(target))
    marker.parent.mkdir(parents=True, exist_ok=True)
    marker.write_text(json.dumps({'root_config': sorted(migrated | affected)})+'\n')


def prepare_pruned_layout(workspace, profile, stage='all'):
    """Rebuild the SLAM install index after removing unused components."""
    if profile not in ('sim', 'hardware'):
        return
    workspace = Path(workspace)
    if stage in ('all', 'lio'):
        (workspace/'install/fast_lio/share/fast_lio/launch/gdb_debug_example.launch').unlink(missing_ok=True)
    if stage not in ('all', 'navigation'):
        return
    marker = workspace/'build/.pm01-slam-components.json'
    if marker.exists() and json.loads(marker.read_text()).get('schema') == 1:
        return
    backup = workspace/'build'/('.retired-slam-components-'+str(time.time_ns()))
    for output in ('build', 'install'):
        path = workspace/output/'slam_toolbox'
        if path.exists() or path.is_symlink():
            target = backup/output/'slam_toolbox'
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.move(str(path), str(target))
    marker.parent.mkdir(parents=True, exist_ok=True)
    marker.write_text(json.dumps({'schema': 1})+'\n')
