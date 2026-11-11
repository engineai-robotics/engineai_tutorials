#!/usr/bin/env python3
"""Publish verified deployment assets to GitHub Releases or a GitLab registry.

Authentication uses GITHUB_TOKEN / GH_TOKEN, or GITLAB_TOKEN / CI_JOB_TOKEN. No token is
written to the release bundle, source, URL, notes or command-line arguments.
"""
import argparse
import hashlib
import http.client
import json
import os
from pathlib import Path
import urllib.parse

from pm01_navigation_runtime.release.package import read_manifest, safe_name, sha256


class GitLab:
    def __init__(self, url, project):
        self.origin = urllib.parse.urlsplit(url.rstrip('/'))
        if self.origin.scheme != 'https' or not self.origin.netloc or self.origin.username:
            raise ValueError('Use an HTTPS GitLab origin without embedded credentials')
        token = os.environ.get('GITLAB_TOKEN') or os.environ.get('CI_JOB_TOKEN')
        if not token:
            raise ValueError('GitLab API authentication required: set GITLAB_TOKEN or CI_JOB_TOKEN locally')
        self.auth = {'PRIVATE-TOKEN' if os.environ.get('GITLAB_TOKEN') else 'JOB-TOKEN': token}
        self.root = self.origin.path.rstrip('/')+'/api/v4/projects/'+urllib.parse.quote(str(project), safe='')

    def connection(self):
        return http.client.HTTPSConnection(self.origin.netloc, timeout=60)

    def request(self, method, path, data=None, accepted=(200,)):
        conn = self.connection()
        headers = dict(self.auth)
        if data is not None:
            data = json.dumps(data).encode(); headers['Content-Type'] = 'application/json'
        try:
            conn.request(method, self.root+path, data, headers)
            response = conn.getresponse(); body = response.read()
            if response.status not in accepted:
                raise RuntimeError('GitLab '+method+' failed: HTTP '+str(response.status))
            return response.status, json.loads(body) if body else None
        finally:
            conn.close()

    def remote_digest(self, path):
        conn = self.connection()
        try:
            conn.request('GET', self.root+path, headers=self.auth)
            response = conn.getresponse()
            if response.status == 404:
                response.read(); return None
            if response.status != 200:
                raise RuntimeError('Package read failed: HTTP '+str(response.status))
            digest = hashlib.sha256()
            for block in iter(lambda: response.read(1024*1024), b''):
                digest.update(block)
            return digest.hexdigest()
        finally:
            conn.close()

    def upload(self, path, file):
        expected = sha256(file)
        existing = self.remote_digest(path)
        if existing is not None:
            if existing != expected:
                raise RuntimeError('Versioned package file already exists with different contents: '+file.name)
            return  # Interrupted uploads may resume only with identical bytes.
        conn = self.connection()
        try:
            conn.putrequest('PUT', self.root+path)
            for name, value in self.auth.items():
                conn.putheader(name, value)
            conn.putheader('Content-Length', str(file.stat().st_size))
            conn.putheader('Content-Type', 'application/octet-stream'); conn.endheaders()
            with file.open('rb') as stream:
                for block in iter(lambda: stream.read(1024*1024), b''):
                    conn.send(block)
            response = conn.getresponse(); response.read()
            if response.status != 201:
                raise RuntimeError('Package upload failed: HTTP '+str(response.status))
        finally:
            conn.close()
        if self.remote_digest(path) != expected:
            raise RuntimeError('Uploaded package checksum mismatch: '+file.name)


def publish(url, project, manifest_path, notes):
    path = Path(manifest_path).expanduser().resolve()
    manifest = read_manifest(path)
    files = [path.parent/p['name'] for p in manifest['parts']]
    files += [path, path.parent/'SHA256SUMS', path.parent/'release_package.py']
    for part in manifest['parts']:
        file = path.parent/part['name']
        if file.stat().st_size != part['size'] or sha256(file) != part['sha256']:
            raise ValueError('Local release file does not match manifest: '+file.name)
    client = GitLab(url, project)
    version = manifest['version']
    status, _ = client.request('GET', '/releases/'+urllib.parse.quote(version, safe=''), accepted=(200, 404))
    if status != 404:
        raise RuntimeError('Release already exists; published versions are not overwritten')
    client.request('GET', '/repository/commits/'+manifest['source_revision'])
    links = []
    for file in files:
        name = safe_name(file.name)
        route = '/packages/generic/pm01-navigation/'+version+'/'+name
        client.upload(route, file)
        links.append(dict(name=name, url=url.rstrip('/')+client.root+route,
                          direct_asset_path='/'+name, link_type='package'))
        print('Verified '+name, flush=True)
    _, result = client.request('POST', '/releases', {
        'name': 'PM01 Navigation '+version, 'tag_name': version,
        'ref': manifest['source_revision'], 'description': Path(notes).read_text(),
        'assets': {'links': links}}, accepted=(201,))
    client.request('GET', '/releases/'+version)
    return dict(ok=True, version=version, source_revision=manifest['source_revision'],
                url=result.get('_links', {}).get('self'), files=len(files))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    target = parser.add_mutually_exclusive_group(required=True)
    target.add_argument('--github-repo', help='Public GitHub owner/repository')
    target.add_argument('--gitlab-url', help='Alternative GitLab server')
    parser.add_argument('--project', help='GitLab project path or ID')
    parser.add_argument('--manifest', required=True); parser.add_argument('--notes', required=True)
    args = parser.parse_args()
    try:
        if args.github_repo == 'engineai-robotics/engineai_fastlio_navigation':
            from pm01_navigation_runtime.release.sources import check_release
            check_release(read_manifest(args.manifest))
        if args.github_repo:
            from pm01_navigation_runtime.release.github import publish as publish_github
            result = publish_github(args.github_repo, args.manifest, args.notes)
        else:
            if not args.project: parser.error('--project is required with --gitlab-url')
            result = publish(args.gitlab_url, args.project, args.manifest, args.notes)
        print(json.dumps(result, ensure_ascii=False))
        return 0
    except Exception as error:
        print(json.dumps(dict(ok=False, error=str(error)), ensure_ascii=False))
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
