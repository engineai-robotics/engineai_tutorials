"""Pinned image identity shared by registry downloads and release publishing."""
import argparse
import hashlib
from importlib import resources
import json
from pathlib import Path
import re
import subprocess

from pm01_navigation_runtime.containers import manager


def load(path=None):
    source = Path(path) if path is not None else resources.files(__package__).joinpath('sources.json')
    value = json.loads(source.read_text())
    if value.get('schema') != 1 or value.get('project') != manager.PROJECT:
        raise ValueError('Unsupported release sources')
    if not re.fullmatch(r'v\d+\.\d+\.\d+(?:-[A-Za-z0-9.-]+)?', value.get('version', '')):
        raise ValueError('Invalid release source version')
    image = value['aliyun']
    for name in ('manifest_digest', 'config_digest'):
        if not re.fullmatch(r'sha256:[0-9a-f]{64}', image.get(name, '')):
            raise ValueError('Invalid registry '+name)
    if image['architecture'] != 'arm64' or not re.fullmatch('[0-9a-f]{64}', image['image_fingerprint']):
        raise ValueError('Invalid ARM64 image identity')
    return value


def github_image(release):
    return release.get('github', release['aliyun'])


def image_info(config):
    return dict(Architecture=config['architecture'], Os=config['os'],
                RootFS={'Layers': config['rootfs']['diff_ids']}, Config=config['config'])


def validate_config(config, expected):
    info = image_info(config)
    labels = info['Config'].get('Labels') or {}
    if (info['Architecture'] != 'arm64' or info['Os'] != 'linux'
            or labels.get('cn.engineai.pm01.project') != manager.PROJECT
            or manager.image_fingerprint(info) != expected['image_fingerprint']
            or labels.get('org.opencontainers.image.revision') != expected['source_revision']
            or labels.get('cn.engineai.pm01.source-sha256') != expected['source_sha256']):
        raise ValueError('Registry image architecture, project or source identity mismatch')
    return info


def inspect_registry(reference):
    if '://' in reference or not re.fullmatch(r'[A-Za-z0-9._/:@-]+', reference):
        raise ValueError('Use a registry/image:tag without a URL scheme or credentials')
    raw = subprocess.check_output(['skopeo', 'inspect', '--raw', 'docker://'+reference], timeout=120)
    digest = 'sha256:'+hashlib.sha256(raw).hexdigest()
    manifest = json.loads(raw)
    if 'config' not in manifest:
        raise ValueError('Publish an ARM64 image manifest, not a multi-platform index')
    repository = reference.rsplit('@', 1)[0].rsplit(':', 1)[0]
    config_raw = subprocess.check_output(['skopeo', 'inspect', '--config', 'docker://'+repository+'@'+digest], timeout=120)
    # inspect --config formats JSON; its output bytes are not the config blob.
    # Skopeo verifies the fetched blob; archive_info verifies the original bytes.
    config = json.loads(config_raw)
    return digest, manifest['config']['digest'], config


def check_release(manifest, sources=None):
    sources = sources or load()
    expected = sources['aliyun']
    if manifest['version'] != sources['version']:
        raise ValueError('Release sources version differs from deployment package')
    for key in ('architecture', 'source_revision', 'image_fingerprint'):
        if manifest[key] != expected[key]:
            raise ValueError('GitHub and Aliyun image identity differs: '+key)
    digest, config_digest, config = inspect_registry(expected['image'])
    if (digest, config_digest) != (expected['manifest_digest'], expected['config_digest']):
        raise ValueError('Aliyun tag changed; regenerate and review release sources before publishing')
    validate_config(config, expected)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bundle', required=True, help='Verified bundle for the same image')
    defaults = load()
    parser.add_argument('--registry-image', default=defaults['aliyun']['image'])
    parser.add_argument('--version', default=defaults['version'])
    parser.add_argument('--output', help='Write reviewed source metadata here; otherwise print it')
    args = parser.parse_args()
    manifest = manager.verify_bundle(Path(args.bundle).expanduser())
    digest, config_digest, config = inspect_registry(args.registry_image)
    validate_config(config, manifest)
    if (config['config'].get('Labels') or {}).get('org.opencontainers.image.version') != args.version.removeprefix('v'):
        raise ValueError('Registry version label differs from release version')
    value = dict(schema=1, project=manager.PROJECT, version=args.version,
        github_manifest='https://github.com/engineai-robotics/engineai_tutorials/releases/download/pm01-navigation-'+args.version+'/release.json',
        aliyun=dict(image=args.registry_image, local_image=manifest['image'], manifest_digest=digest, config_digest=config_digest,
            **{k: manifest[k] for k in ('architecture', 'image_fingerprint', 'source_revision', 'source_sha256')}))
    payload = json.dumps(value, indent=2)+'\n'
    if args.output:
        Path(args.output).write_text(payload)
    else:
        print(payload, end='')
    return 0
