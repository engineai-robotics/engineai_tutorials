"""Apply deployment fields to an existing robot runtime using only Python stdlib.

This module is also sent over the authenticated SSH connection, not installed
into or imported from the navigation image. It never starts a container or ROS.
"""
import fcntl
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import sys
import tempfile
import xml.etree.ElementTree as ET


def replace_file(path, content):
    """Back up changed bytes and atomically replace a user-owned configuration."""
    path = Path(path)
    if path.is_symlink():
        raise ValueError('Configuration symlink left untouched: '+str(path))
    original = path.read_bytes() if path.exists() else None
    encoded = content.encode()
    if original == encoded:
        return dict(path=str(path), changed=False)
    path.parent.mkdir(parents=True, exist_ok=True)
    backup = None
    if original is not None:
        backup = path.with_name(path.name+'.before-deployment-'+hashlib.sha256(original).hexdigest()[:12]+'.bak')
        try:
            fd = os.open(backup, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
        except FileExistsError:
            if backup.read_bytes() != original:
                raise RuntimeError('Existing deployment backup differs: '+str(backup))
        else:
            with os.fdopen(fd, 'wb') as stream:
                stream.write(original)
    with tempfile.NamedTemporaryFile(mode='w', dir=path.parent, delete=False) as stream:
        temporary = Path(stream.name)
        stream.write(content)
    try:
        temporary.chmod(0o600)
        if (path.read_bytes() if path.exists() else None) != original:
            raise RuntimeError('Configuration changed during apply; retry: '+str(path))
        temporary.replace(path)
    finally:
        temporary.unlink(missing_ok=True)
    return dict(path=str(path), changed=True, backup=str(backup) if backup else None)


def merge_dds(original, generated):
    """Update generated network lists while retaining other transport settings."""
    if original is None:
        return generated
    ns = '{http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles}'
    ET.register_namespace('', ns[1:-1])
    root, desired = ET.fromstring(original), ET.fromstring(generated)
    transports = root.findall(ns+'transport_descriptors/'+ns+'transport_descriptor')
    transport = next((t for t in transports if t.findtext(ns+'transport_id') == 'pm01_udp'), None)
    participants = [p for p in root.findall(ns+'participant')
                    if p.get('profile_name') == 'pm01' and p.get('is_default_profile') == 'true']
    if transport is None or len(participants) != 1:
        raise ValueError('Custom DDS profile left untouched; select a generated pm01 profile first')
    rtps = participants[0].find(ns+'rtps')
    builtin = rtps.find(ns+'builtin') if rtps is not None else None
    if builtin is None:
        raise ValueError('DDS profile has no discovery configuration')
    source_transport = desired.find(ns+'transport_descriptors/'+ns+'transport_descriptor')
    source_builtin = desired.find(ns+'participant/'+ns+'rtps/'+ns+'builtin')
    for parent, source, tag in ((transport, source_transport, 'interfaceWhiteList'),
                                (builtin, source_builtin, 'initialPeersList')):
        previous = parent.findall(ns+tag)
        position = list(parent).index(previous[0]) if previous else len(parent)
        for old in previous:
            parent.remove(old)
        parent.insert(position, source.find(ns+tag))
    # Repair only the known empty listener override from the original generator.
    for entry in builtin.findall(ns+'metatrafficUnicastLocatorList'):
        if len(entry) == 1 and entry[0].tag == ns+'locator' and not len(entry[0]) and not entry[0].attrib:
            builtin.remove(entry)
    ET.indent(root)
    return ET.tostring(root, encoding='unicode')+'\n'


def apply_robot(request, runtime=None):
    if runtime is None:
        from pm01_navigation_runtime.containers import manager as module
        runtime = module.Runtime(request['robot']['container_config'], request['robot']['machine_id'])
    if runtime.identity != request['robot']['machine_id']:
        raise ValueError('Robot identity mismatch; configurations unchanged')
    with runtime.path.with_suffix('.lock').open('a') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as error:
            raise RuntimeError('Another project operation is running; retry after it completes') from error
        path = Path(runtime.session_config)
        existing = json.loads(path.read_text()) if path.exists() else {}
        if not isinstance(existing, dict):
            raise ValueError('Existing session configuration must be a JSON object')
        if existing and existing.get('expected_machine_id') != runtime.identity:
            raise ValueError('Existing session belongs to a different robot')
        desired = dict(existing) if existing else dict(
            workspace='/opt/pm01_navigation', profile='hardware', expected_machine_id=runtime.identity,
            map_root='~/.local/share/pm01_navigation/maps', launch_args={},
            state_root=str(Path.home()/'.local/state/pm01_navigation'/
                           ('docker-'+hashlib.sha256(str(runtime.path).encode()).hexdigest()[:16])))
        profile = Path(existing.get('dds_profile') or path.parent/'fastdds-robot.xml').expanduser().resolve()
        if not profile.is_relative_to(Path.home()/'.config/pm01_navigation'):
            raise ValueError('Keep DDS configuration inside the mounted ~/.config/pm01_navigation directory')
        desired.update(ros_domain_id=request['network']['ros_domain_id'], dds_profile=str(profile))
        # Compute every result before writing, so malformed files leave both intact.
        original_xml = profile.read_text() if profile.exists() else None
        dds = merge_dds(original_xml, request['robot_dds'])
        serialized = json.dumps(desired, indent=2)+'\n'
        changed = existing != desired or original_xml != dds
        info = runtime.owned()
        if changed and info and info['State']['Running']:
            status = runtime.session('status')
            if status.get('running') is not False:
                raise RuntimeError('Stop this project navigation session before applying changed configuration')
        results = [replace_file(profile, dds), replace_file(path, serialized)]
        return dict(ok=True, changed=any(r['changed'] for r in results), files=results,
                    session_config=str(path), image=runtime.config['image'], navigation_started=False)


if __name__ == '__main__':
    try:
        print(json.dumps(apply_robot(json.load(sys.stdin)), ensure_ascii=False))
    except Exception as error:
        print(json.dumps(dict(ok=False, error=str(error)), ensure_ascii=False))
        raise SystemExit(1)
