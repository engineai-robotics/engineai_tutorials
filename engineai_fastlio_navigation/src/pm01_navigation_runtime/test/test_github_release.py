"""Public release downloads must survive GitHub's signed CDN redirects."""
import pytest
from pm01_navigation_runtime.release import package, github


def test_parts_use_canonical_url_after_manifest_redirect(tmp_path, monkeypatch):
    url = 'https://github.com/engineai-robotics/engineai_tutorials/releases/download/pm01-navigation-v1.0.0/release.json'
    calls = []
    def download(source, path, **kwargs):
        calls.append(source)
        path.write_bytes(b'data')
        return 'https://release-assets.githubusercontent.com/signed/manifest?signature=one'
    monkeypatch.setattr(package, 'download', download)
    monkeypatch.setattr(package, 'read_manifest', lambda _: dict(version='v1.0.0', architecture='arm64',
        archive_sha256='a'*64, parts=[dict(name='part0001', size=4, sha256='hash')]))
    monkeypatch.setattr(package, 'sha256', lambda _: 'hash')
    monkeypatch.setattr(package, 'unpack', lambda *args: {})
    package.fetch(url, tmp_path/'out', tmp_path/'cache')
    assert calls[1] == url.replace('release.json', 'part0001')


def test_public_github_does_not_receive_gitlab_credentials(tmp_path, monkeypatch):
    monkeypatch.setenv('PM01_DOWNLOAD_TOKEN', 'private-deploy-token')
    monkeypatch.setenv('GITLAB_TOKEN', 'private-api-token')
    class Response:
        headers = {'Content-Type': 'application/octet-stream'}
        url = 'https://release-assets.githubusercontent.com/file'
        def __enter__(self): return self
        def __exit__(self, *args): pass
        def read(self, *args): return b''
    class Opener:
        def open(self, request, **kwargs):
            assert not any(k.lower().endswith('token') for k, v in request.header_items())
            return Response()
    monkeypatch.setattr(package.urllib.request, 'build_opener', lambda *args: Opener())
    package.download('https://github.com/org/repo/releases/download/v1.0.0/release.json', tmp_path/'file')


@pytest.mark.parametrize("corrupt", [False, True])
def test_github_assets_verified_before_publication(tmp_path, monkeypatch, corrupt):
    part = tmp_path/'part'; part.write_bytes(b'data')
    for name in ('release_package.py', 'SHA256SUMS', 'release.json', 'notes.md'):
        (tmp_path/name).write_text(name)
    manifest = dict(parts=[dict(name='part', size=4, sha256=package.sha256(part))],
                    source_revision='a'*40, version='v1.0.0')
    monkeypatch.setattr(github, 'read_manifest', lambda _: manifest)
    calls = []
    class Fake:
        def __init__(self, repo): pass
        def listing(self, path): return []
        def request(self, method, path, data=None, accepted=(200,)):
            calls.append((method, path))
            if path == '': return 200, dict(private=False)
            if path == '/commits/v1.0.0': return 404, None
            if path.startswith('/commits/'): return 200, dict(sha='a'*40)
            if method == 'POST':
                assert data['draft']; return 201, dict(id=7)
            assert len([c for c in calls if c[0] == 'upload']) == 4
            assert data['draft'] is False
            return 200, dict(html_url='https://github.com/org/repo/releases/tag/v1.0.0')
        def upload(self, rid, file):
            calls.append(('upload', file.name))
            return dict(size=file.stat().st_size, digest='bad' if corrupt else 'sha256:'+package.sha256(file), state='uploaded')
    monkeypatch.setattr(github, 'GitHub', Fake)
    if corrupt:
        with pytest.raises(RuntimeError, match='remains a draft'):
            github.publish('org/repo', tmp_path/'release.json', tmp_path/'notes.md')
        assert not any(method == 'PATCH' for method, path in calls)
    else:
        assert github.publish('org/repo', tmp_path/'release.json', tmp_path/'notes.md')['ok']
