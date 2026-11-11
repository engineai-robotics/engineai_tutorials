"""Prepare a verified ARM64 deployment bundle without a Docker daemon."""
import hashlib
import json
from pathlib import Path
import subprocess
import tarfile
import tempfile

from . import bundle, manager
from ..release import sources


def archive_info(path, expected):
    """Read and verify a Docker archive without extracting or running its files."""
    with tarfile.open(path) as archive:
        def member(name):
            if Path(name).is_absolute() or '..' in Path(name).parts:
                raise ValueError('Unsafe Docker archive member')
            value = archive.getmember(name)
            if not value.isfile():
                raise ValueError('Docker archive member is not a regular file')
            return archive.extractfile(value)
        manifests = json.load(member('manifest.json'))
        if len(manifests) != 1:
            raise ValueError('Expected exactly one ARM64 image')
        manifest = manifests[0]
        raw = member(manifest['Config']).read()
        if 'sha256:'+hashlib.sha256(raw).hexdigest() != expected['config_digest']:
            raise ValueError('Docker archive configuration checksum mismatch')
        config = json.loads(raw)
        info = sources.validate_config(config, expected)
        layers = info['RootFS']['Layers']
        if len(layers) != len(manifest['Layers']):
            raise ValueError('Docker archive layer count mismatch')
        for name, digest in zip(manifest['Layers'], layers):
            with member(name) as stream:
                checksum = hashlib.sha256()
                for block in iter(lambda: stream.read(1024*1024), b''):
                    checksum.update(block)
            if 'sha256:'+checksum.hexdigest() != digest:
                raise ValueError('Docker archive layer checksum mismatch')
        tag = expected.get('local_image', expected['image'])
        aliases = {tag}
        if '/' not in tag:
            aliases.add('docker.io/library/'+tag)
        if not aliases.intersection(manifest.get('RepoTags') or []):
            raise ValueError('Docker archive image tag mismatch: '+str(manifest.get('RepoTags')))
        return info


def tools_identity(workspace):
    import pm01_navigation_runtime
    root = Path(pm01_navigation_runtime.__file__).parent
    files = [(str(p.relative_to(root)), p) for p in root.rglob('*')
             if p.is_file() and p.suffix in ('.py', '.json') and '__pycache__' not in p.parts]
    files += [(name, Path(workspace)/'scripts'/name)
              for name in ('_bootstrap.py', 'pm01_container.py', 'container_bundle.py')]
    checksum = hashlib.sha256()
    for name, path in sorted(files):
        checksum.update(name.encode()+b'\0'+path.read_bytes()+b'\0')
    return checksum.hexdigest()


def prepare(workspace, cache, release=None):
    release = release or sources.load()
    expected = release['aliyun']
    cache = Path(cache)
    tools_sha256 = tools_identity(workspace)
    key = hashlib.sha256(json.dumps(dict(image=expected, tools=tools_sha256), sort_keys=True).encode()).hexdigest()
    target = cache/'bundles'/('aliyun-'+key)
    if target.exists():
        try:
            manifest = manager.verify_bundle(target)
            if (manifest['image_fingerprint'] != expected['image_fingerprint']
                    or manifest.get('registry_digest') != expected['manifest_digest']
                    or manifest.get('tools_sha256') != tools_sha256):
                raise ValueError('Cached registry bundle identity mismatch')
            return target, manifest
        except (ValueError, RuntimeError, OSError, KeyError):
            # Only our generated cache is discarded; valid OCI blobs are retained.
            import shutil
            shutil.rmtree(target)
    target.parent.mkdir(parents=True, exist_ok=True)
    blobs = cache/'registry'/expected['manifest_digest'].split(':', 1)[1]
    blobs.parent.mkdir(parents=True, exist_ok=True)
    repository = expected['image'].rsplit(':', 1)[0]
    reference = repository+'@'+expected['manifest_digest']
    # The OCI directory retains complete content-addressed blobs across retries.
    # Skopeo checks registry digests and reuses already downloaded layers.
    print('从阿里云下载 ARM64 镜像；已完成的镜像层会复用。', flush=True)
    subprocess.run(['skopeo', '--override-os', 'linux', '--override-arch', 'arm64',
                    'copy', '--retry-times', '2', 'docker://'+reference,
                    'oci:'+str(blobs)+':image'], check=True, timeout=3600)
    with tempfile.TemporaryDirectory(prefix='.aliyun-', dir=target.parent) as temporary:
        stage = Path(temporary)/'bundle'
        stage.mkdir()
        subprocess.run(['skopeo', 'copy', 'oci:'+str(blobs)+':image',
                        'docker-archive:'+str(stage/'image.tar')+':'+expected.get('local_image', expected['image'])],
                       check=True, timeout=1800)
        archive_info(stage/'image.tar', expected)
        bundle.export_runtime(Path(workspace), stage)
        if tools_identity(workspace) != tools_sha256:
            raise ValueError('Deployment tools changed during download; rerun with the retained image cache')
        manifest = dict(schema=1, project=manager.PROJECT, version=release['version'],
            image=expected.get('local_image', expected['image']), architecture='arm64', image_id=expected['config_digest'],
            image_fingerprint=expected['image_fingerprint'], source_revision=expected['source_revision'],
            source_sha256=expected['source_sha256'], registry_digest=expected['manifest_digest'],
            tools_sha256=tools_sha256,
            files={str(p.relative_to(stage)): manager.digest(p) for p in stage.rglob('*') if p.is_file()})
        try:
            manifest['tools_source_revision'] = subprocess.check_output(
                ['git', '-C', str(workspace), 'rev-parse', 'HEAD'], text=True, stderr=subprocess.DEVNULL).strip()
        except (OSError, subprocess.CalledProcessError):
            pass  # Source archives still have an exact tools content checksum.
        (stage/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
        manager.verify_bundle(stage)
        stage.rename(target)
    return target, manifest
