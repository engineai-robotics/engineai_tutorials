#!/usr/bin/env python3
"""Versioned offline release files; fetch/verify/unpack need only Python 3."""
import argparse
import base64
import hashlib
import importlib.util
import io
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tarfile
import tempfile
import zipfile
import urllib.error
import urllib.parse
import urllib.request



def sha256(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024*1024), b''):
            h.update(block)
    return h.hexdigest()


def safe_name(name):
    if not isinstance(name, str) or not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_.-]*', name):
        raise ValueError('Invalid release file name')
    return name


def read_manifest(path):
    value = json.loads(Path(path).read_text())
    if value.get('schema') != 1 or value.get('project') != 'fastlio_navigation':
        raise ValueError('Unsupported release manifest')
    if not re.fullmatch(r'v\d+\.\d+\.\d+(?:-[A-Za-z0-9.-]+)?', value.get('version', '')):
        raise ValueError('Expected a version such as v1.0.0')
    if not re.fullmatch('[0-9a-f]{64}', value.get('archive_sha256', '')):
        raise ValueError('Invalid archive checksum')
    for part in value['parts']:
        safe_name(part['name'])
        if not re.fullmatch('[0-9a-f]{64}', part['sha256']) or part['size'] <= 0:
            raise ValueError('Invalid release part checksum or length')
    if not value['parts'] or len({p['name'] for p in value['parts']}) != len(value['parts']):
        raise ValueError('Empty or duplicate release parts')
    return value


def pack(bundle, output, version, part_mib=256, *, workspace):
    if not re.fullmatch(r'v\d+\.\d+\.\d+(?:-[A-Za-z0-9.-]+)?', version) or part_mib < 1:
        raise ValueError('Use a semantic version and a positive part size')
    bundle, output = Path(bundle).expanduser().resolve(), Path(output).expanduser().resolve()
    if output.exists() or output.is_relative_to(workspace):
        raise ValueError('Use a new release directory outside the source tree')
    from pm01_navigation_runtime.containers import manager as runtime
    manifest = runtime.verify_bundle(bundle)
    revision = subprocess.check_output(['git', '-C', str(workspace), 'rev-parse', 'HEAD'], text=True).strip()
    if manifest.get('source_revision') != revision:
        raise ValueError('Bundle image must be built from the exact committed release source')
    if subprocess.check_output(['git', '-C', str(workspace), 'status', '--porcelain'], text=True).strip():
        raise ValueError('Pack from a clean checkout; keep unrelated local work in its own workspace')
    output.mkdir(parents=True)
    with tempfile.TemporaryDirectory(prefix='pm01-release-') as temp:
        temp = Path(temp)
        source_tar = temp/'source.tar'
        subprocess.run(['git', '-C', str(workspace), 'archive', '--format=tar', '-o', str(source_tar), revision], check=True)
        source = temp/'source.tar.gz'
        with tarfile.open(source_tar) as inp, tarfile.open(source, 'w:gz') as out:
            for member in inp:
                out.addfile(member, inp.extractfile(member) if member.isfile() else None)
            member = tarfile.TarInfo('SOURCE_VERSION'); content = (revision+'\n').encode()
            member.size = len(content); out.addfile(member, io.BytesIO(content))
        archive = temp/'deployment.tar.gz'
        with tarfile.open(archive, 'w:gz', compresslevel=3) as stream:
            for name in sorted(manifest['files']):
                stream.add(bundle/name, arcname=name, recursive=False)
            stream.add(bundle/'manifest.json', arcname='manifest.json')
            stream.add(source, arcname='source.tar.gz')
        name = 'pm01-navigation-'+version+'-linux-'+manifest['architecture']+'.tar.gz'
        parts = []
        with archive.open('rb') as stream:
            index = 1
            while True:
                block = stream.read(part_mib*1024*1024)
                if not block:
                    break
                path = output/(name+'.part'+str(index).zfill(4)); path.write_bytes(block)
                parts.append(dict(name=path.name, size=path.stat().st_size, sha256=sha256(path)))
                index += 1
        info = dict(schema=1, project='fastlio_navigation', version=version,
                    architecture=manifest['architecture'], source_revision=revision,
                    image=manifest['image'], image_fingerprint=manifest['image_fingerprint'],
                    archive=name, archive_sha256=sha256(archive), parts=parts)
    (output/'release.json').write_text(json.dumps(info, indent=2)+'\n')
    # A downloadable helper remains one Python file. Its embedded runtime is
    # generated from the package, never maintained as a second implementation.
    import pm01_navigation_runtime
    payload = io.BytesIO()
    module_root = Path(pm01_navigation_runtime.__file__).parent
    with zipfile.ZipFile(payload, 'w', zipfile.ZIP_DEFLATED) as archive:
        for path in sorted(module_root.rglob('*')):
            if path.is_file() and '__pycache__' not in path.parts and path.suffix in ('.py', '.json'):
                archive.writestr('pm01_navigation_runtime/'+str(path.relative_to(module_root)), path.read_bytes())
    (output/'release_package.py').write_text(
        '#!/usr/bin/env python3\nimport base64, sys, tempfile\nfrom pathlib import Path\n'
        'with tempfile.TemporaryDirectory(prefix="pm01-release-") as folder:\n'
        '    payload = Path(folder)/"runtime.zip"\n'
        '    payload.write_bytes(base64.b64decode('+repr(base64.b64encode(payload.getvalue()).decode())+'))\n'
        '    sys.path.insert(0, str(payload))\n'
        '    from pm01_navigation_runtime.release.package import main\n'
        '    raise SystemExit(main(Path.cwd()))\n')
    files = [output/p['name'] for p in parts]+[output/'release.json', output/'release_package.py']
    (output/'SHA256SUMS').write_text(''.join(sha256(p)+'  '+p.name+'\n' for p in files))
    return dict(ok=True, directory=str(output), version=version, parts=len(parts))


def unpack(manifest_path, output):
    manifest_path, output = Path(manifest_path).expanduser().resolve(), Path(output).expanduser().resolve()
    manifest = read_manifest(manifest_path)
    for part in manifest['parts']:
        path = manifest_path.parent/part['name']
        if path.stat().st_size != part['size'] or sha256(path) != part['sha256']:
            raise ValueError('Release part checksum mismatch: '+part['name'])
    if output.exists():
        marker = output/'release.json'
        if marker.is_file() and json.loads(marker.read_text()) == manifest:
            from pm01_navigation_runtime.containers import manager as runtime
            runtime.verify_bundle(output)
            return dict(ok=True, output=str(output), reused=True)
        raise ValueError('Output already exists; use a new version directory')
    output.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='.pm01-unpack-', dir=output.parent) as temp:
        temp = Path(temp); archive = temp/'archive.tar.gz'
        with archive.open('wb') as stream:
            for part in manifest['parts']:
                with (manifest_path.parent/part['name']).open('rb') as inp:
                    shutil.copyfileobj(inp, stream)
        if sha256(archive) != manifest['archive_sha256']:
            raise ValueError('Combined archive checksum mismatch')
        destination = temp/'deployment'; destination.mkdir()
        with tarfile.open(archive) as stream:
            for member in stream.getmembers():
                path = Path(member.name)
                if path.is_absolute() or '..' in path.parts or not (member.isdir() or member.isfile()):
                    raise ValueError('Unsafe archive member: '+member.name)
            stream.extractall(destination)
        shutil.copy2(manifest_path, destination/'release.json')
        destination.rename(output)
    return dict(ok=True, output=str(output), version=manifest['version'])


class SafeRedirect(urllib.request.HTTPRedirectHandler):
    def redirect_request(self, req, fp, code, msg, headers, newurl):
        if urllib.parse.urlsplit(newurl).scheme != 'https':
            raise ValueError('发布文件重定向必须使用 HTTPS')
        redirected = super().redirect_request(req, fp, code, msg, headers, newurl)
        if urllib.parse.urlsplit(req.full_url).netloc != urllib.parse.urlsplit(newurl).netloc:
            for name in ('Private-token', 'Job-token', 'Deploy-token'):
                redirected.remove_header(name)
        return redirected


def download(url, destination, credential_host=None):
    parsed = urllib.parse.urlsplit(url)
    if parsed.scheme != 'https':
        raise ValueError('Release downloads require HTTPS')
    if parsed.username or parsed.password:
        raise ValueError('下载地址中不要包含账号或凭据，请使用环境变量认证')
    headers = {}
    allow_credentials = ('/api/v4/projects/' in parsed.path and
                         (credential_host is None or parsed.netloc == credential_host))
    if allow_credentials and os.environ.get('PM01_DOWNLOAD_TOKEN'):
        headers['DEPLOY-TOKEN'] = os.environ['PM01_DOWNLOAD_TOKEN']
    elif allow_credentials and os.environ.get('GITLAB_TOKEN'):
        headers['PRIVATE-TOKEN'] = os.environ['GITLAB_TOKEN']
    elif allow_credentials and os.environ.get('CI_JOB_TOKEN'):
        headers['JOB-TOKEN'] = os.environ['CI_JOB_TOKEN']
    if urllib.parse.urlsplit(url).scheme not in ('https', 'http'):
        raise ValueError('下载地址应为原始 HTTP(S) 文件链接，不要粘贴 Markdown 链接格式')
    request = urllib.request.Request(url, headers=headers)
    destination = Path(destination)
    temporary = destination.with_suffix(destination.suffix+'.partial')
    try:
        with urllib.request.build_opener(SafeRedirect()).open(request, timeout=60) as response, temporary.open('wb') as out:
            prefix = response.read(512)
            if 'text/html' in response.headers.get('Content-Type', '').lower() or prefix.lstrip().lower().startswith((b'<!doctype html', b'<html')):
                raise ValueError('下载地址返回网页而非发布文件，请使用 Release 资产的直接下载链接。私有 GitLab 软件包需使用有权限的下载令牌及 Generic Package API 地址')
            out.write(prefix)
            shutil.copyfileobj(response, out)
            final_url = response.url
        temporary.replace(destination)
        return final_url
    except urllib.error.HTTPError as error:
        temporary.unlink(missing_ok=True)
        if error.code in (401, 403):
            raise ValueError('下载无权限（HTTP '+str(error.code)+'）；请核对令牌及项目包读取权限') from error
        if error.code == 404:
            raise ValueError('发布文件不存在或当前账号无权读取（HTTP 404）；核对版本和文件地址') from error
        raise ValueError('下载失败（HTTP '+str(error.code)+'）；已校验分卷保留，可重复执行下载命令') from error
    except Exception:
        temporary.unlink(missing_ok=True)
        raise


def fetch(url, output, cache=None, *, content_addressed=False, expected_identity=None):
    cache = Path(cache).expanduser() if cache else Path.home()/'.cache/pm01_navigation/releases'
    cache.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='manifest-', dir=cache) as temp:
        path = Path(temp)/'release.json'
        download(url, path)
        try:
            manifest = read_manifest(path)
        except json.JSONDecodeError as error:
            raise ValueError('release.json 不是有效 JSON；请使用清单原始文件地址，而非 Releases 页面地址') from error
        if expected_identity and any(manifest.get(k) != v for k, v in expected_identity.items()):
            raise ValueError('发布清单与当前源码选择的镜像不一致，请更新源码或检查发布内容；未下载镜像分卷')
        print('发布版本：'+manifest['version']+'；自动处理 '+str(len(manifest['parts']))+' 个分卷', file=sys.stderr)
        digest = manifest['archive_sha256']
        directory = cache/(manifest['version']+'-'+manifest['architecture']+'-'+digest[:16])
        if content_addressed:
            output = Path(output).with_name(Path(output).name+'-'+digest[:16])
        directory.mkdir(exist_ok=True)
        shutil.copy2(path, directory/'release.json')
    for part in manifest['parts']:
        path = directory/part['name']
        if path.is_file() and path.stat().st_size == part['size'] and sha256(path) == part['sha256']:
            print('复用已校验分卷：'+part['name'], file=sys.stderr)
            continue
        print('下载分卷：'+part['name'], file=sys.stderr)
        download(urllib.parse.urljoin(url, part['name']), path,
                 credential_host=urllib.parse.urlsplit(url).netloc)
        if path.stat().st_size != part['size'] or sha256(path) != part['sha256']:
            raise ValueError('Downloaded release part failed verification: '+part['name'])
    print('分卷校验完成，正在合并解包……', file=sys.stderr)
    result = unpack(directory/'release.json', output)
    result.update(version=manifest['version'], image=manifest.get('image'), source_revision=manifest.get('source_revision'))
    return result


def main(workspace):
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='action', required=True)
    create = sub.add_parser('pack'); create.add_argument('--bundle', required=True)
    create.add_argument('--output', required=True); create.add_argument('--version', required=True)
    create.add_argument('--part-mib', type=int, default=256)
    extract = sub.add_parser('unpack'); extract.add_argument('--manifest', required=True)
    extract.add_argument('--output', required=True)
    get = sub.add_parser('fetch'); get.add_argument('--manifest-url', required=True)
    get.add_argument('--output', required=True); get.add_argument('--cache')
    args = parser.parse_args()
    try:
        if args.action == 'pack':
            result = pack(args.bundle, args.output, args.version, args.part_mib, workspace=workspace)
        elif args.action == 'unpack':
            result = unpack(args.manifest, args.output)
        else:
            result = fetch(args.manifest_url, args.output, args.cache)
        print(json.dumps(result, ensure_ascii=False))
        return 0
    except Exception as error:
        print(json.dumps(dict(ok=False, error=str(error)), ensure_ascii=False))
        return 1


