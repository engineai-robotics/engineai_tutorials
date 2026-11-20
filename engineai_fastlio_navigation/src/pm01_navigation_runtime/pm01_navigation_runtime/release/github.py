"""Publish verified deployment files to a GitHub Release via a draft."""
import http.client
import json
import os
from pathlib import Path
import re
from urllib.parse import quote

from .package import read_manifest, safe_name, sha256


class GitHub:
    def __init__(self, repository):
        if not re.fullmatch(r'[A-Za-z0-9_.-]+/[A-Za-z0-9_.-]+', repository):
            raise ValueError('Use owner/repository for --github-repo')
        token = os.environ.get('GITHUB_TOKEN') or os.environ.get('GH_TOKEN')
        if not token:
            raise ValueError('Set GITHUB_TOKEN with repository Contents write permission')
        self.root = '/repos/' + repository
        self.headers = {'Authorization': 'Bearer '+token, 'Accept': 'application/vnd.github+json',
                        'User-Agent': 'pm01-navigation-publisher', 'X-GitHub-Api-Version': '2022-11-28'}

    def request(self, method, path, data=None, accepted=(200,)):
        conn = http.client.HTTPSConnection('api.github.com', timeout=60)
        headers = dict(self.headers)
        if data is not None:
            data = json.dumps(data).encode(); headers['Content-Type'] = 'application/json'
        try:
            conn.request(method, self.root+path, body=data, headers=headers)
            response = conn.getresponse(); body = response.read()
            if response.status not in accepted:
                raise RuntimeError('GitHub '+method+' failed: HTTP '+str(response.status))
            return response.status, json.loads(body) if body else None
        finally:
            conn.close()

    def listing(self, path):
        result = []
        for page in range(1, 1001):
            _, items = self.request('GET', path+'?per_page=100&page='+str(page))
            result.extend(items)
            if len(items) < 100: return result
        raise RuntimeError('GitHub pagination limit exceeded')

    def upload(self, release_id, file):
        conn = http.client.HTTPSConnection('uploads.github.com', timeout=120)
        try:
            conn.putrequest('POST', self.root+'/releases/'+str(release_id)+'/assets?name='+quote(file.name, safe=''))
            for key, value in self.headers.items(): conn.putheader(key, value)
            conn.putheader('Content-Type', 'application/octet-stream')
            conn.putheader('Content-Length', str(file.stat().st_size)); conn.endheaders()
            with file.open('rb') as stream:
                for block in iter(lambda: stream.read(1024*1024), b''): conn.send(block)
            response = conn.getresponse(); body = response.read()
            if response.status != 201:
                raise RuntimeError('GitHub asset upload failed: HTTP '+str(response.status))
            return json.loads(body)
        finally:
            conn.close()


def publish(repository, manifest_path, notes):
    path = Path(manifest_path).expanduser().resolve()
    manifest = read_manifest(path)
    files = [path.parent/safe_name(p['name']) for p in manifest['parts']]
    files += [path.parent/'release_package.py', path.parent/'SHA256SUMS', path]
    for part, file in zip(manifest['parts'], files):
        if file.stat().st_size != part['size'] or sha256(file) != part['sha256']:
            raise ValueError('Release part checksum mismatch: '+file.name)
    expected = {f.name: (f.stat().st_size, 'sha256:'+sha256(f)) for f in files}
    notes_text = Path(notes).read_text()
    client = GitHub(repository)
    _, repo = client.request('GET', '')
    if repo.get('private'):
        raise ValueError('The default setup wizard requires a public GitHub repository')
    revision = manifest['source_revision']; version = manifest['version']
    _, commit = client.request('GET', '/commits/'+quote(revision, safe=''))
    if commit['sha'] != revision: raise ValueError('Source commit must exist in the GitHub repository')
    status, tagged = client.request('GET', '/commits/'+quote(version, safe=''), accepted=(200, 404, 422))
    if status == 200 and tagged['sha'] != revision:
        raise ValueError('Version tag points to a different source commit')
    releases = client.listing('/releases')
    release = next((r for r in releases if r['tag_name'] == version), None)
    if release and (not release['draft'] or release['target_commitish'] != revision):
        raise ValueError('Release exists; only a matching unpublished draft can resume')
    if release is None:
        _, release = client.request('POST', '/releases', dict(tag_name=version, target_commitish=revision,
            name='PM01 Navigation '+version, body=notes_text, draft=True, prerelease=False), accepted=(201,))
    rid = release['id']
    assets = client.listing('/releases/'+str(rid)+'/assets')
    by_name = {a['name']: a for a in assets}
    if len(by_name) != len(assets) or set(by_name)-set(expected):
        raise ValueError('Draft contains unexpected assets; review it before publishing')
    for file in files:
        asset = by_name.get(file.name) or client.upload(rid, file)
        if (asset.get('size'), asset.get('digest')) != expected[file.name] or asset.get('state') != 'uploaded':
            raise RuntimeError('GitHub asset verification failed; release remains a draft: '+file.name)
        print('Verified '+file.name, flush=True)
    _, release = client.request('PATCH', '/releases/'+str(rid), dict(draft=False, body=notes_text))
    return dict(ok=True, version=version, source_revision=revision, url=release['html_url'], files=len(files))
