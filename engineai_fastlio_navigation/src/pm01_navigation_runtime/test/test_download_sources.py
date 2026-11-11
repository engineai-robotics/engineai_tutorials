"""Download transports must produce the same verified deployment contract."""
import copy
import hashlib
import io
import json
from pathlib import Path
import subprocess
import sys
import tarfile
from types import SimpleNamespace

import pytest
from pm01_navigation_runtime.containers import registry, manager
from pm01_navigation_runtime.deployment import setup_hardware as setup
from pm01_navigation_runtime.release import sources


def wizard(tmp_path, **kwargs):
    args = dict(bundle=None, manifest_url=None, download_source=None)
    args.update(kwargs)
    return setup.Wizard(SimpleNamespace(**args), root=tmp_path/'config', cache=tmp_path/'cache', workspace=tmp_path)


def fixture_image(tmp_path):
    layer = b'a layer'
    config = dict(architecture='arm64', os='linux',
        rootfs=dict(diff_ids=['sha256:'+hashlib.sha256(layer).hexdigest()]),
        config=dict(Labels={'cn.engineai.pm01.project':'fastlio_navigation',
                           'org.opencontainers.image.revision':'a'*40,
                           'cn.engineai.pm01.source-sha256':'b'*64}, Env=['A=B']))
    raw = json.dumps(config).encode()
    expected = dict(image='registry.example/pm01:v1.0.0', architecture='arm64',
        local_image='pm01-navigation:v1.0.0-arm64', manifest_digest='sha256:'+'c'*64, config_digest='sha256:'+hashlib.sha256(raw).hexdigest(),
        image_fingerprint=manager.image_fingerprint(sources.image_info(config)),
        source_revision='a'*40, source_sha256='b'*64)
    def write(path, changed=None):
        entries = {'config.json':raw, 'layer.tar':layer,
                   'manifest.json':json.dumps([dict(Config='config.json', Layers=['layer.tar'],
                                                   RepoTags=[expected['local_image']])]).encode()}
        entries.update(changed or {})
        with tarfile.open(path, 'w') as t:
            for name, data in entries.items():
                member = tarfile.TarInfo(name); member.size = len(data)
                t.addfile(member, io.BytesIO(data))
    path = tmp_path/'fixture.tar'; write(path)
    return path, expected, write


def test_first_choice_is_required_and_remembered(tmp_path, monkeypatch):
    w = wizard(tmp_path)
    answers = iter(['', 'wrong', '1'])
    monkeypatch.setattr('builtins.input', lambda _: next(answers))
    assert w.select_download_source() == 'aliyun'
    monkeypatch.setattr('builtins.input', lambda _: pytest.fail('Unexpected prompt'))
    assert wizard(tmp_path).select_download_source() == 'aliyun'
    assert wizard(tmp_path, download_source='github').select_download_source() == 'github'
    assert wizard(tmp_path).select_download_source() == 'github'


def test_no_default_for_noninteractive_first_run(tmp_path, monkeypatch):
    def eof(_): raise EOFError()
    monkeypatch.setattr('builtins.input', eof)
    with pytest.raises(setup.SetupError, match='--download-source'):
        wizard(tmp_path).select_download_source()


@pytest.mark.parametrize('option', ['bundle', 'manifest_url'])
def test_explicit_offline_or_manifest_skips_selection(tmp_path, monkeypatch, option):
    w = wizard(tmp_path, **{option:'custom'})
    monkeypatch.setattr(w, 'select_download_source', lambda: pytest.fail('Unexpected selection'))
    monkeypatch.setattr(w, 'fetch_bundle', lambda: ('existing', {}))
    assert w.bundle() == ('existing', {})
    assert not (w.root/'download-source.json').exists()


@pytest.mark.parametrize('args', [
    ['--bundle','a','--download-source','aliyun'],
    ['--manifest-url','https://example.test/release.json','--download-source','github'],
    ['--bundle','a','--manifest-url','https://example.test/release.json']])
def test_conflicting_sources_rejected_before_side_effects(tmp_path, monkeypatch, args):
    monkeypatch.setattr(sys, 'argv', ['setup', *args])
    with pytest.raises(SystemExit) as error: setup.main(tmp_path)
    assert error.value.code == 2


def test_download_failure_requires_choice_before_switch(tmp_path, monkeypatch):
    w = wizard(tmp_path, download_source='aliyun')
    monkeypatch.setattr(setup.shutil, 'which', lambda _: '/skopeo')
    calls = []
    def fail(*args):
        calls.append('aliyun'); raise RuntimeError('network disconnected')
    monkeypatch.setattr(registry, 'prepare', fail)
    expected = sources.github_image(sources.load())
    monkeypatch.setattr(w, 'fetch_bundle', lambda: (calls.append('github') or 'github', expected))
    monkeypatch.setattr(sys.stdin, 'isatty', lambda: True)
    answers = iter(['1', '2', '2'])  # retry, then explicitly switch to GitHub
    monkeypatch.setattr('builtins.input', lambda _: next(answers))
    assert w.bundle()[0] == 'github'
    assert calls == ['aliyun', 'aliyun', 'github']
    assert json.loads((w.root/'download-source.json').read_text())['source'] == 'github'


def test_noninteractive_failure_does_not_switch(tmp_path, monkeypatch):
    w = wizard(tmp_path, download_source='aliyun')
    monkeypatch.setattr(setup.shutil, 'which', lambda _: '/skopeo')
    def fail(*args): raise RuntimeError('offline')
    monkeypatch.setattr(registry, 'prepare', fail)
    monkeypatch.setattr(sys.stdin, 'isatty', lambda: False)
    monkeypatch.setattr(w, 'fetch_bundle', lambda: pytest.fail('Silent source switch'))
    with pytest.raises(RuntimeError, match='offline'): w.bundle()


def test_github_uses_its_own_image_identity(tmp_path, monkeypatch):
    w = wizard(tmp_path, download_source='github')
    monkeypatch.setattr(sys.stdin, 'isatty', lambda: False)
    release = sources.load()
    aliyun = release['aliyun']
    github = dict(aliyun, image_fingerprint='0'*64, source_revision='1'*40, source_sha256='2'*64)
    monkeypatch.setattr(sources, 'load', lambda: dict(release, github=github))
    from pm01_navigation_runtime.release import package

    def fetch(*args, expected_identity, **kwargs):
        assert expected_identity == dict(version=release['version'], architecture='arm64',
                                        image_fingerprint=github['image_fingerprint'],
                                        source_revision=github['source_revision'])
        return {'output': str(tmp_path)}

    monkeypatch.setattr(package, 'fetch', fetch)
    monkeypatch.setattr(manager, 'verify_bundle', lambda _: github)
    assert w.bundle() == (tmp_path, github)
    monkeypatch.setattr(manager, 'verify_bundle', lambda _: aliyun)
    with pytest.raises(setup.SetupError, match='不一致'): w.bundle()


@pytest.mark.parametrize('damage', ['config', 'layer', 'tag', 'architecture', 'project'])
def test_archive_rejects_invalid_image(tmp_path, damage):
    path, expected, write = fixture_image(tmp_path)
    if damage == 'config': write(path, {'config.json':b'{}'})
    elif damage == 'layer': write(path, {'layer.tar':b'damaged'})
    elif damage == 'tag': expected['local_image'] = 'wrong/image:v1.0.0'
    else:
        with tarfile.open(path) as t: config = json.load(t.extractfile('config.json'))
        if damage == 'architecture': config['architecture'] = 'amd64'
        else: config['config']['Labels']['cn.engineai.pm01.project'] = 'other'
        raw = json.dumps(config).encode()
        expected['config_digest'] = 'sha256:'+hashlib.sha256(raw).hexdigest()
        write(path, {'config.json':raw})
    with pytest.raises(ValueError): registry.archive_info(path, expected)


def test_prepare_cache_tools_and_partial_failure(tmp_path, monkeypatch):
    fixture, expected, write = fixture_image(tmp_path)
    release = dict(version='v1.0.0', aliyun=expected)
    monkeypatch.setattr(registry, 'tools_identity', lambda _: 'tools-one')
    monkeypatch.setattr(registry.bundle, 'export_runtime', lambda w, p: (p/'tool.py').write_text('tool'))
    monkeypatch.setattr(registry.subprocess, 'check_output', lambda *a, **k: 'tools-revision')
    calls = []
    failure = [True]
    def run(args, **kwargs):
        assert args[0] == 'skopeo'
        calls.append(args)
        if failure[0]: raise subprocess.CalledProcessError(1, args)
        if args[-1].startswith('docker-archive:'):
            output = args[-1].split(':', 2)[1]; write(output)
    monkeypatch.setattr(registry.subprocess, 'run', run)
    with pytest.raises(subprocess.CalledProcessError): registry.prepare(tmp_path, tmp_path/'cache', release)
    assert not list((tmp_path/'cache/bundles').glob('aliyun-*'))
    failure[0] = False
    out, manifest = registry.prepare(tmp_path, tmp_path/'cache', release)
    assert manifest['source_revision'] != manifest['tools_source_revision']
    assert manager.verify_bundle(out)['image_fingerprint'] == expected['image_fingerprint']
    before = len(calls)
    assert registry.prepare(tmp_path, tmp_path/'cache', release)[0] == out
    assert len(calls) == before
    (out/'image.tar').write_bytes(b'corrupt')
    registry.prepare(tmp_path, tmp_path/'cache', release)
    assert len(calls) == before+2  # repair invalid cache, retaining layer directory
    monkeypatch.setattr(registry, 'tools_identity', lambda _: 'tools-two')
    new, _ = registry.prepare(tmp_path, tmp_path/'cache', release)
    assert new != out and out.exists()
    assert calls[-2][-1] == calls[-4][-1]  # same image blobs reused for new tools
    newer = copy.deepcopy(release); newer['aliyun']['manifest_digest'] = 'sha256:'+'d'*64
    changed, _ = registry.prepare(tmp_path, tmp_path/'cache', newer)
    assert changed not in (out, new)
    assert calls[-2][-1] != calls[-4][-1]


def test_publication_rejects_same_tag_different_contents(tmp_path, monkeypatch):
    _, expected, _ = fixture_image(tmp_path)
    selected = dict(version='v1.0.0', aliyun=expected)
    manifest = dict(expected, version='v1.0.0')
    monkeypatch.setattr(sources, 'inspect_registry', lambda _: ('sha256:'+'0'*64, expected['config_digest'], {}))
    with pytest.raises(ValueError, match='tag changed'): sources.check_release(manifest, selected)
    manifest['source_revision'] = '0'*40
    with pytest.raises(ValueError, match='identity differs'): sources.check_release(manifest, selected)


def test_sources_generator_compares_bundle_before_writing(tmp_path, monkeypatch):
    path, expected, _ = fixture_image(tmp_path)
    with tarfile.open(path) as archive:
        config = json.load(archive.extractfile('config.json'))
    config['config']['Labels']['org.opencontainers.image.version'] = '1.0.0'
    expected['image_fingerprint'] = manager.image_fingerprint(sources.image_info(config))
    manifest = dict(expected, image=expected['local_image'])
    monkeypatch.setattr(manager, 'verify_bundle', lambda _: manifest)
    monkeypatch.setattr(sources, 'inspect_registry', lambda _: (expected['manifest_digest'], expected['config_digest'], config))
    out = tmp_path/'sources.json'
    monkeypatch.setattr(sys, 'argv', ['sources', '--bundle', str(tmp_path), '--output', str(out)])
    assert sources.main() == 0
    generated = sources.load(out)
    assert generated['aliyun']['local_image'] == manifest['image']
    assert generated['aliyun']['image_fingerprint'] == expected['image_fingerprint']
    out.unlink(); manifest['source_revision'] = '0'*40
    with pytest.raises(ValueError, match='identity mismatch'): sources.main()
    assert not out.exists()


def test_changed_tools_during_download_do_not_publish_bundle(tmp_path, monkeypatch):
    _, expected, write = fixture_image(tmp_path)
    versions = iter(['before', 'after'])
    monkeypatch.setattr(registry, 'tools_identity', lambda _: next(versions))
    monkeypatch.setattr(registry.bundle, 'export_runtime', lambda *a: None)
    def run(args, **kwargs):
        if args[-1].startswith('docker-archive:'):
            write(args[-1].split(':', 2)[1])
    monkeypatch.setattr(registry.subprocess, 'run', run)
    with pytest.raises(ValueError, match='tools changed'):
        registry.prepare(tmp_path, tmp_path/'cache', dict(version='v1.0.0', aliyun=expected))
    assert not list((tmp_path/'cache/bundles').iterdir())


def test_release_sources_available_in_standalone_zip(tmp_path):
    import zipfile
    import pm01_navigation_runtime
    root = Path(pm01_navigation_runtime.__file__).parent
    payload = tmp_path/'runtime.zip'
    with zipfile.ZipFile(payload, 'w') as zipped:
        for p in root.rglob('*'):
            if p.is_file() and p.suffix in ('.py', '.json') and '__pycache__' not in p.parts:
                zipped.write(p, 'pm01_navigation_runtime/'+str(p.relative_to(root)))
    code = 'import sys; sys.path.insert(0, sys.argv[1]); from pm01_navigation_runtime.release.sources import load; print(load()["version"])'
    result = subprocess.check_output([sys.executable, '-I', '-c', code, str(payload)], text=True)
    assert result.strip() == sources.load()['version']


def test_mismatched_release_stops_before_large_download(tmp_path, monkeypatch):
    from pm01_navigation_runtime.release import package
    calls = []
    def download(url, path, **kwargs):
        calls.append(url); path.write_text('{}')
    monkeypatch.setattr(package, 'download', download)
    monkeypatch.setattr(package, 'read_manifest', lambda _: dict(version='v1.0.0', image_fingerprint='old'))
    with pytest.raises(ValueError, match='未下载镜像分卷'):
        package.fetch('https://example.test/release.json', tmp_path/'out', tmp_path/'cache',
                      expected_identity=dict(image_fingerprint='new'))
    assert calls == ['https://example.test/release.json']
    assert not (tmp_path/'out').exists()


def test_docker_archive_accepts_skopeo_canonical_short_name(tmp_path):
    path, expected, write = fixture_image(tmp_path)
    write(path, {'manifest.json':json.dumps([dict(Config='config.json', Layers=['layer.tar'],
        RepoTags=['docker.io/library/'+expected['local_image']])]).encode()})
    assert registry.archive_info(path, expected)['Architecture'] == 'arm64'
