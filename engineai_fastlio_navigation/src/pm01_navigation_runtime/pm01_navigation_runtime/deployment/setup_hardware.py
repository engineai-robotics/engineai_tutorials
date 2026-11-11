#!/usr/bin/env python3
"""Prepare a native developer PC and PM01, stopping before sensor startup."""
import argparse
import base64
import fcntl
import getpass
import hashlib
import ipaddress
import json
import os
from pathlib import Path
import re
import shlex
import shutil
import socket
import subprocess
import sys
import tempfile
import time
import uuid
from urllib.parse import urlsplit
from pm01_navigation_runtime.containers.bundle import standalone_program
from pm01_navigation_runtime.release import sources as release_sources

CONFIG = Path.home()/'.config/pm01_navigation'
CACHE = Path.home()/'.cache/pm01_navigation'
RELEASE_SOURCES = release_sources.load()
RELEASE = RELEASE_SOURCES['version']
MANIFEST = RELEASE_SOURCES['github_manifest']


class SetupError(RuntimeError):
    pass


def execute(argv, *, capture=False, check=True, timeout=None, input=None, env=None):
    result = subprocess.run(argv, text=True, input=input, env=env,
                            stdout=subprocess.PIPE if capture else None,
                            stderr=subprocess.PIPE if capture else None, timeout=timeout)
    if check and result.returncode:
        detail = (result.stderr or result.stdout or '')[-1500:] if capture else ''
        raise SetupError(detail or '命令未完成，请根据上方提示修正后重新运行向导')
    return result


def json_result(output, require_ok=True):
    for line in reversed(output.splitlines()):
        try:
            value = json.loads(line)
        except ValueError:
            continue
        if isinstance(value, dict):
            if require_ok and not value.get('ok', True):
                raise SetupError(value.get('error', '远程准备失败'))
            return value
    raise SetupError('未收到可识别的结果，请检查 SSH 连接；不会将超时解释为操作成功')


def confirm(prompt, default=False):
    answer = input(prompt+(' [Y/n]：' if default else ' [y/N]：')).strip().lower()
    return answer in ('y', 'yes', '是') or (not answer and default)


def native_ubuntu():
    release = Path('/etc/os-release')
    values = {}
    if release.is_file():
        for line in release.read_text().splitlines():
            if '=' in line:
                key, value = line.split('=', 1)
                values[key] = value.strip('"')
    if values.get('ID') != 'ubuntu' or values.get('VERSION_ID') != '22.04':
        raise SetupError('本向导支持 Ubuntu 22.04 开发电脑；机器人端通过 SSH 准备')
    if os.geteuid() == 0:
        raise SetupError('请使用普通开发账号运行向导；需要 sudo 的步骤会单独提示')


def clean_build_environment():
    env = dict(os.environ)
    for name in ('PYTHONHOME', 'PYTHONPATH', 'AMENT_PREFIX_PATH', 'COLCON_PREFIX_PATH',
                 'CMAKE_PREFIX_PATH', 'LD_LIBRARY_PATH', 'CONDA_PREFIX', 'CONDA_DEFAULT_ENV',
                 'GITLAB_TOKEN', 'GITHUB_TOKEN', 'GH_TOKEN', 'PM01_DOWNLOAD_TOKEN', 'CI_JOB_TOKEN'):
        env.pop(name, None)
    env['PATH'] = '/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin'
    return env


def write_file(path, content, mode=0o600, backup=True):
    from pm01_navigation_runtime.deployment.hardware_apply import replace_file
    if backup:
        result = replace_file(path, content)
    else:
        path = Path(path)
        if path.is_symlink():
            raise SetupError('进度文件是符号链接，保留原文件：'+str(path))
        path.parent.mkdir(parents=True, exist_ok=True)
        with tempfile.NamedTemporaryFile(mode='w', dir=path.parent, delete=False) as stream:
            stream.write(content)
            temp = Path(stream.name)
        try:
            temp.chmod(mode)
            temp.replace(path)
        finally:
            temp.unlink(missing_ok=True)
        result = dict(path=str(path), changed=True)
    Path(path).chmod(mode)
    return result


def prepare_private_bundle(source, destination):
    """Recover an interrupted per-device copy without altering shared files."""
    from pm01_navigation_runtime.containers import manager as pm01_container
    try:
        pm01_container.verify_bundle(destination)
        return
    except (OSError, ValueError, RuntimeError, KeyError):
        pass
    def link_or_copy(src, dst):
        try:
            return os.link(src, dst)
        except OSError:
            return shutil.copy2(src, dst)
    temporary = destination.with_name(destination.name+'.prepare-'+uuid.uuid4().hex[:8])
    try:
        shutil.copytree(source, temporary, copy_function=link_or_copy)
        (temporary/'manifest.json').unlink()
        shutil.copy2(source/'manifest.json', temporary/'manifest.json')
        pm01_container.verify_bundle(temporary)
        # Only this tool's private, reproducible staging copy is replaced.
        if destination.exists():
            shutil.rmtree(destination)
        temporary.replace(destination)
    finally:
        shutil.rmtree(temporary, ignore_errors=True)


def records(root):
    result = []
    for path in sorted((root/'robots').glob('*/record.json')):
        try:
            value = json.loads(path.read_text())
            if not re.fullmatch('[a-f0-9]{32}', value['machine_id']):
                continue
            result.append(value)
        except (OSError, ValueError, KeyError):
            continue
    return result


def choose_target(root, supplied=None):
    if supplied:
        return supplied
    known = records(root)
    if known:
        print('\n已登记的机器人（也可以输入新地址）：')
        for index, row in enumerate(known, 1):
            print('  {}. {}  {}  身份 {}'.format(index, row['hostname'], row['address'], row['machine_id'][:12]))
    answer = input('机器人地址，例如 192.168.0.162；账号默认 ubuntu：').strip()
    if answer.isdigit() and 1 <= int(answer) <= len(known):
        row = known[int(answer)-1]
        return row['ssh_host']
    if not answer:
        raise SetupError('未填写机器人地址；下次直接重新运行本命令即可')
    return answer


def parse_target(value):
    if not re.fullmatch(r'[A-Za-z0-9_][A-Za-z0-9_.@-]*', value) or value.count('@') > 1:
        raise SetupError('请输入 IPv4、主机名或 SSH 别名，可使用 ubuntu@地址')
    # ssh -G also resolves an existing user's Host alias without changing it.
    config = execute(['ssh', '-G', value], capture=True).stdout
    values = {}
    identities = []
    for line in config.splitlines():
        key, _, content = line.partition(' ')
        if key == 'identityfile':
            identities.append(str(Path(content).expanduser()))
        else:
            values.setdefault(key, content)
    user = values.get('user', 'ubuntu') if '@' in value or values.get('hostname') != value else 'ubuntu'
    host = values.get('hostname', value.rsplit('@', 1)[-1])
    address = str(ipaddress.IPv4Address(socket.gethostbyname(host)))
    if values.get('proxycommand') not in (None, 'none') or values.get('proxyjump') not in (None, 'none'):
        raise SetupError('此入口准备原生局域网直连；SSH 跳板／USB 隧道请使用对应接入说明')
    known = values.get('userknownhostsfile', '').split()
    custom_hosts = str(Path(known[0]).expanduser()) if len(known) == 1 else ''
    host_key_alias = values.get('hostkeyalias', '')
    if host_key_alias == 'none':
        host_key_alias = ''
    return dict(target=value if values.get('hostname') != value.rsplit('@', 1)[-1] else user+'@'+address,
                user=user, address=address, identities=identities, supplied=value,
                port=values.get('port', '22'), known_hosts=custom_hosts, host_key_alias=host_key_alias)


def usable_key(path):
    if not path.is_file() or not Path(str(path)+'.pub').is_file():
        return False
    unlocked = execute(['ssh-keygen', '-y', '-P', '', '-f', str(path)],
                       capture=True, check=False)
    if unlocked.returncode == 0:
        return True
    agent = execute(['ssh-add', '-l'], capture=True, check=False)
    fingerprint = execute(['ssh-keygen', '-lf', str(path)+'.pub'], capture=True, check=False)
    fields = fingerprint.stdout.split()
    return agent.returncode == 0 and len(fields) > 1 and fields[1] in agent.stdout.split()


class Connection:
    def __init__(self, target, root, port='22', *, workspace):
        self.workspace = Path(workspace)
        self.target, self.root = target, root
        self.folder = root/'ssh'
        self.folder.mkdir(parents=True, exist_ok=True)
        self.folder.chmod(0o700)
        self.known_hosts = None
        self.key = None
        self.socket = self.folder/('control-'+uuid.uuid4().hex[:12])
        self.alias = None
        self.host_key_alias = ''
        self.address = target.rsplit('@', 1)[-1]
        if not str(port).isdigit() or not 1 <= int(port) <= 65535:
            raise SetupError('Invalid SSH port')
        self.port = str(port)

    def options(self, batch=True, multiplex=True):
        args = ['ssh', '-T', '-o', 'StrictHostKeyChecking=ask' if not batch else 'StrictHostKeyChecking=yes',
                '-o', 'ConnectTimeout=8', '-o', 'ServerAliveInterval=5', '-o', 'ServerAliveCountMax=2', '-p', self.port]
        if batch:
            args += ['-o', 'BatchMode=yes']
        else:
            args += ['-o', 'BatchMode=no']
        if multiplex:
            args += ['-o', 'ControlMaster=auto', '-o', 'ControlPersist=300', '-o', 'ControlPath='+str(self.socket)]
        else:
            args += ['-o', 'ControlPath=none']
        if self.known_hosts:
            args += ['-o', 'UserKnownHostsFile='+str(self.known_hosts)]
        if self.host_key_alias:
            args += ['-o', 'HostKeyAlias='+self.host_key_alias]
        if self.key:
            args += ['-i', str(self.key), '-o', 'IdentitiesOnly=yes']
        return args

    def call(self, command, data=None, timeout=60):
        return execute(self.options()+[self.alias or self.target, command], capture=True,
                       input=data, timeout=timeout).stdout

    def identify(self):
        source = standalone_program('deployment.setup_remote')
        command = 'python3 -c '+shlex.quote(source)+' identity'
        result = execute(self.options(batch=False)+[self.target, command], capture=False,
                         check=False, input=None)
        if result.returncode:
            if not confirm('SSH 未通过。如上方提示主机密钥变化，且此地址确已更换机器人，是否另存主机身份重试？'):
                raise SetupError('SSH 登录未完成；请核对账号、密码或密钥后重跑')
            self.known_hosts = self.folder/('known_hosts-'+uuid.uuid4().hex[:12])
            execute(self.options(batch=False)+[self.target, command])
        # The first command uses native SSH prompts. Read identity again over
        # the authenticated control connection, not from ssh-keyscan output.
        return json_result(self.call(command, '{}'))

    def close(self):
        if self.socket.exists():
            execute(['ssh', '-p', self.port, '-o', 'ControlPath='+str(self.socket), '-O', 'exit', self.target],
                    capture=True, check=False, timeout=8)

    def register(self, identity, candidates):
        previous = self.root/'robots'/identity['machine_id']/'record.json'
        old = json.loads(previous.read_text()) if previous.is_file() else {}
        paths = [old.get('key', ''), *candidates, str(Path.home()/'.ssh/pm01_navigation_ed25519'),
                 str(self.folder/'id_ed25519')]
        key = next((Path(p) for p in paths if p and usable_key(Path(p))), None)
        if key is None:
            key = self.folder/'id_ed25519'
            if key.exists():
                raise SetupError('已有项目密钥目前无法免交互使用；保留原文件。可先 ssh-add 解锁，或补齐其公钥后重跑')
            print('没有可供免交互使用的密钥，创建独立项目密钥；已有密钥保留。')
            execute(['ssh-keygen', '-q', '-t', 'ed25519', '-N', '', '-f', str(key), '-C', 'pm01-navigation'])
        public = key.with_name(key.name+'.pub').read_text().strip()
        program = '''import json,os
from pathlib import Path
key=json.load(__import__('sys').stdin)['public_key'].strip()
if '\\n' in key or len(key.split())<2:raise SystemExit('Invalid public key')
folder=Path.home()/'.ssh';folder.mkdir(mode=0o700,exist_ok=True)
path=folder/'authorized_keys'
lines=path.read_text().splitlines() if path.exists() else []
if key.split()[:2] not in [line.split()[:2] for line in lines]:
 with path.open('a') as stream:stream.write(('\\n' if lines else '')+key+'\\n')
path.chmod(0o600)
print(json.dumps(dict(ok=True)))
'''
        json_result(self.call('python3 -c '+shlex.quote(program), json.dumps(dict(public_key=public))))
        alias = old.get('ssh_host') or 'pm01-'+identity['machine_id'][:12]
        if not re.fullmatch('[A-Za-z0-9_.-]+', alias):
            raise SetupError('Invalid existing robot alias')
        entry = {'address': self.address, 'user': identity['user'],
                 'key': str(key), 'known_hosts': str(self.known_hosts) if self.known_hosts else '',
                 'host_key_alias': self.host_key_alias, 'port': self.port}
        entries_path = self.folder/'hosts.json'
        entries = json.loads(entries_path.read_text()) if entries_path.exists() else {}
        entries[alias] = entry
        lines = ['# Generated PM01 aliases; other SSH hosts are not managed here.']
        for name, row in sorted(entries.items()):
            lines += ['Host '+name, '    HostName '+row['address'], '    User '+row['user'], '    Port '+row.get('port', '22'),
                      '    IdentityFile "'+row['key']+'"', '    IdentitiesOnly yes',
                      '    StrictHostKeyChecking yes', '    ConnectTimeout 8',
                      '    ServerAliveInterval 5', '    ServerAliveCountMax 2']
            if row['known_hosts']:
                lines += ['    UserKnownHostsFile "'+row['known_hosts']+'"']
            if row.get('host_key_alias'):
                lines += ['    HostKeyAlias '+row['host_key_alias']]
        write_file(entries_path, json.dumps(entries, indent=2)+'\n')
        managed = self.folder/'config'
        write_file(managed, '\n'.join(lines)+'\n')
        user_config = Path.home()/'.ssh/config'
        user_config.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
        content = user_config.read_text() if user_config.exists() else ''
        include = 'Include "'+str(managed)+'"'
        if include not in content.splitlines():
            write_file(user_config, include+'\n'+content)
        self.key, self.alias = key, alias
        self.close()
        # A fresh login proves that the key is registered and later also
        # refreshes supplementary groups after an admin action.
        actual = json_result(self.call('python3 -c '+shlex.quote(standalone_program('deployment.setup_remote'))+' identity', '{}'))
        if actual['machine_id'] != identity['machine_id']:
            raise SetupError('Robot identity changed after reconnecting')
        return alias, entry

    def helper(self, action, request, timeout=90):
        command = 'python3 -c '+shlex.quote(standalone_program('deployment.setup_remote'))+' '+action
        return json_result(self.call(command, json.dumps(request), timeout))

    def copy(self, source, destination, timeout=None):
        ssh = shlex.join(self.options())
        execute(['rsync', '-a', '-s', '--partial', '--partial-dir=.rsync-partial',
                 '-e', ssh, str(source), (self.alias or self.target)+':'+destination], timeout=timeout)


def route_address(robot_address):
    rows = json.loads(execute(['ip', '-j', '-4', 'route', 'get', robot_address], capture=True).stdout)
    if not rows or not rows[0].get('prefsrc'):
        raise SetupError('未能获取通往机器人的开发机地址，请检查网口或 Wi-Fi 连接')
    address = ipaddress.IPv4Address(rows[0]['prefsrc'])
    if address.is_loopback or rows[0].get('dev', '').startswith(('docker', 'veth', 'br-')):
        raise SetupError('通往机器人的是回环或容器网桥，请检查实际网络连接')
    return str(address)


def activate(root, workspace, console):
    if not (workspace/'install/local_setup.bash').is_file() or not console.is_file():
        raise SetupError('构建结果或配置缺失，保留原 active.env')
    import yaml
    settings = yaml.safe_load(console.read_text()) or {}
    content = ('# Generated after successful PM01 preparation.\n'
               'if [[ ! -f '+shlex.quote(str(workspace/'scripts/env.sh'))+' ]]; then\n'
               '  echo "工作空间已移动，请在新源码目录重新运行 setup_hardware.sh" >&2\n'
               '  return 1\nfi\n'
               'source '+shlex.quote(str(workspace/'scripts/env.sh'))+' --config '+shlex.quote(str(console))+' || return $?\n'
               'export PM01_ROBOT_SSH='+shlex.quote(settings.get('ssh_host', ''))+'\n'
               'export PM01_CONSOLE_CONFIG='+shlex.quote(str(console))+'\n'
               'export PM01_DEPLOYMENT_CONFIG='+shlex.quote(str(console.parent/'deployment.yaml'))+'\n')
    return write_file(root/'active.env', content)


class Wizard:
    def __init__(self, args, root=CONFIG, cache=CACHE, *, workspace):
        self.workspace = Path(workspace)
        self.args, self.root, self.cache = args, Path(root), Path(cache)
        self.connection = None
        self.stage = '开始'
        self.state = {}
        self.folder = None
        self.warnings = []

    def step(self, name):
        self.stage = name
        print('\n== '+name+' ==', flush=True)
        self.save('running')

    def save(self, status, error=None):
        if self.folder:
            self.state.update(stage=self.stage, status=status, updated_at=time.time())
            if error:
                self.state['error'] = error
            write_file(self.folder/'setup-state.json', json.dumps(self.state, ensure_ascii=False, indent=2)+'\n', backup=False)

    def desktop(self):
        self.step('准备开发电脑')
        native_ubuntu()
        for package in ('client', 'bringup', 'console'):
            if not (self.workspace/('src/pm01_navigation_'+package)/'package.xml').is_file():
                raise SetupError('源码目录不完整，请确认仓库已完整获取后重跑')
        env = clean_build_environment()
        names = execute(['bash', str(self.workspace/'scripts/install_dependencies.sh'), 'desktop', '--list-packages'],
                        capture=True, env=env).stdout.splitlines()
        installed = execute(['dpkg-query', '-W', '-f=${Package} ${db:Status-Status}\n', *names],
                            capture=True, check=False, env=env).stdout.splitlines()
        found = {line.split()[0] for line in installed if line.endswith(' installed')}
        missing = sorted(set(names)-found)
        if missing:
            print('安装缺少的开发依赖：'+', '.join(missing))
            execute(['bash', str(self.workspace/'scripts/install_dependencies.sh'), 'desktop'], env=env)
        else:
            print('已有 ROS 2 与开发依赖，直接复用。')
        execute(['bash', str(self.workspace/'scripts/build.sh'), 'desktop'], env=env)

    def select_download_source(self, force=False):
        names = {'aliyun': '国内网络：阿里云镜像', 'github': '海外网络：GitHub 部署包'}
        preference = self.root/'download-source.json'
        selected = None
        if not force:
            selected = self.args.download_source
            if not selected and preference.is_file():
                try:
                    selected = json.loads(preference.read_text()).get('source')
                except (ValueError, OSError):
                    pass
        while not isinstance(selected, str) or selected not in names:
            print('选择下载来源：1. '+names['aliyun']+'；2. '+names['github'])
            try:
                answer = input('请输入 1 或 2：').strip()
            except EOFError as error:
                raise SetupError('非交互运行请指定 --download-source aliyun 或 github') from error
            selected = {'1': 'aliyun', '2': 'github'}.get(answer)
        print('下载来源：'+names[selected]+'；可用 --download-source aliyun|github 切换。')
        write_file(preference, json.dumps(dict(source=selected))+'\n', backup=False)
        return selected

    def bundle(self):
        self.step('准备机载部署包')
        if self.args.bundle or self.args.manifest_url:
            return self.fetch_bundle()
        source = self.select_download_source()
        while True:
            try:
                if source == 'github':
                    result = self.fetch_bundle()
                    manifest = result[1]
                    expected = release_sources.github_image(release_sources.load())
                    if any(manifest.get(k) != expected[k] for k in
                           ('architecture', 'image_fingerprint', 'source_revision', 'source_sha256')):
                        raise SetupError('GitHub 部署包与当前发布镜像不一致，请更新源码或检查发布内容')
                    return result
                if not shutil.which('skopeo'):
                    print('安装镜像下载工具 Skopeo；开发电脑无需安装 Docker。')
                    admin = [] if os.geteuid() == 0 else ['sudo']
                    execute([*admin, 'apt-get', 'update'])
                    execute([*admin, 'apt-get', 'install', '--no-install-recommends', '--no-remove', '-y', 'skopeo'])
                from pm01_navigation_runtime.containers.registry import prepare
                return prepare(self.workspace, self.cache)
            except Exception as error:
                print('下载未完成：'+str(error)+'；已校验的缓存会保留。')
                if not sys.stdin.isatty():
                    raise
                answer = input('1. 重试当前来源  2. 切换来源  3. 退出 [3]：').strip()
                if answer == '2':
                    source = self.select_download_source(force=True)
                elif answer != '1':
                    raise SetupError('下载已退出；可重跑向导继续') from error

    def fetch_bundle(self):
        from pm01_navigation_runtime.release import package as release_package
        from pm01_navigation_runtime.containers import manager as pm01_container
        manifest_url = self.args.manifest_url or MANIFEST
        expected_identity = None
        if not self.args.manifest_url and not self.args.bundle:
            release = release_sources.load()
            image = release_sources.github_image(release)
            expected_identity = dict(version=release['version'], **{k: image[k]
                for k in ('architecture', 'source_revision', 'image_fingerprint')})
        if self.args.bundle:
            path = Path(self.args.bundle).expanduser().resolve(strict=True)
            manifest = pm01_container.verify_bundle(path)
        else:
            key = RELEASE if manifest_url == MANIFEST else 'source-'+hashlib.sha256(manifest_url.encode()).hexdigest()[:12]
            path = self.cache/'bundles'/key
            credentials = self.root/'download.json'
            original_token = os.environ.get('PM01_DOWNLOAD_TOKEN')
            token = original_token
            if (not token and credentials.is_file() and
                    '/api/v4/projects/' in urlsplit(manifest_url).path):
                if credentials.stat().st_mode & 0o077:
                    raise SetupError('下载凭据文件权限过宽，请执行 chmod 600 '+str(credentials))
                saved = json.loads(credentials.read_text())
                if saved.get('host') == urlsplit(manifest_url).hostname:
                    token = saved.get('deploy_token')
            if token:
                os.environ['PM01_DOWNLOAD_TOKEN'] = token
            try:
                result = release_package.fetch(manifest_url, path, self.cache/'releases',
                    content_addressed=True, expected_identity=expected_identity)
            except ValueError as error:
                if ('/api/v4/projects/' not in urlsplit(manifest_url).path or
                        not any(word in str(error) for word in ('无权限', '登录页面'))):
                    raise
                print('私有发布包需要项目只读下载凭据，无需创建个人 API 令牌。')
                token = getpass.getpass('粘贴维护者提供的只读部署令牌：').strip()
                if not token:
                    raise SetupError('未提供下载凭据；已校验分卷仍保留，可稍后重跑')
                os.environ['PM01_DOWNLOAD_TOKEN'] = token
                try:
                    result = release_package.fetch(manifest_url, path, self.cache/'releases',
                    content_addressed=True, expected_identity=expected_identity)
                except Exception:
                    os.environ.pop('PM01_DOWNLOAD_TOKEN', None)
                    raise
                write_file(credentials, json.dumps(dict(deploy_token=token, host=urlsplit(manifest_url).hostname))+'\n')
            finally:
                if original_token:
                    os.environ['PM01_DOWNLOAD_TOKEN'] = original_token
                else:
                    os.environ.pop('PM01_DOWNLOAD_TOKEN', None)
            path = Path(result['output'])
            manifest = pm01_container.verify_bundle(path)
        if manifest.get('architecture') != 'arm64':
            raise SetupError('需要 ARM64 PM01 部署包，未向机器人安装任何内容')
        return path, manifest

    def connect(self):
        self.step('连接并登记机器人')
        target = parse_target(choose_target(self.root, self.args.host))
        self.connection = Connection(target['target'], self.root, target['port'], workspace=self.workspace)
        self.connection.address = target['address']
        self.connection.host_key_alias = target.get('host_key_alias', '')
        if target.get('known_hosts'):
            self.connection.known_hosts = Path(target['known_hosts'])
        # Reuse a per-device host-key record for a previously verified endpoint.
        known = sorted(records(self.root), key=lambda row: row.get('updated_at', 0), reverse=True)
        selected = [row for row in known if row['ssh_host'] == target['supplied']]
        for row in selected or known:
            if row.get('address') == target['address'] and row.get('user') == target['user']:
                self.connection.known_hosts = Path(row['known_hosts']) if row.get('known_hosts') else None
                self.connection.host_key_alias = row.get('host_key_alias', '')
                if row.get('key'):
                    self.connection.key = Path(row['key'])
                break
        identity = self.connection.identify()
        if identity['os'] != 'ubuntu' or identity['version'] != '22.04' or identity['architecture'] not in ('aarch64', 'arm64'):
            raise SetupError('目标不是支持的 Ubuntu 22.04 ARM64 PM01；未部署')
        self.folder = self.root/'robots'/identity['machine_id']
        self.folder.mkdir(parents=True, exist_ok=True)
        print('机器人：{}  地址：{}  身份：{}'.format(identity['hostname'], target['address'], identity['machine_id']))
        if not (self.folder/'record.json').is_file() and not confirm('确认这是本次要配置的机器人？'):
            raise SetupError('未确认目标，机器人配置未更改')
        alias, entry = self.connection.register(identity, target['identities'])
        record = dict(machine_id=identity['machine_id'], hostname=identity['hostname'], updated_at=time.time(),
                      ssh_host=alias, **entry)
        write_file(self.folder/'record.json', json.dumps(record, ensure_ascii=False, indent=2)+'\n')
        self.state.update(machine_id=identity['machine_id'], address=target['address'], ssh_host=alias)
        self.identity = identity
        self.request = dict(machine_id=identity['machine_id'], uid=identity['uid'], user=identity['user'],
                            container_tool='~/pm01_navigation_deploy/pm01_container.py',
                            container_config='~/.config/pm01_navigation/container.json')
        self.address = target['address']
        self.alias = alias

    def inspect(self, **extra):
        return self.connection.helper('inspect', dict(self.request, **extra))

    def admin(self, info, bundle):
        if info['docker_access']:
            return
        self.step('准备机器人 Docker 与账号权限')
        request = dict(self.request, bundle=str(bundle))
        remote_dir = self.identity['home']+'/.cache/pm01_navigation/setup'
        self.connection.call('mkdir -p '+shlex.quote(remote_dir))
        with tempfile.TemporaryDirectory(prefix='pm01-setup-admin-') as temp:
            path = Path(temp)/'request.json'
            path.write_text(json.dumps(request));path.chmod(0o600)
            self.connection.copy(path, remote_dir+'/request.json')
            helper = self.cache/'setup_remote.py'
            helper.parent.mkdir(parents=True, exist_ok=True)
            helper.write_text(standalone_program('deployment.setup_remote'))
            self.connection.copy(helper, remote_dir+'/setup_remote.py')
        cmd = 'sudo python3 '+shlex.quote(remote_dir+'/setup_remote.py')+' admin '+shlex.quote(remote_dir+'/request.json')
        argv = self.connection.options()
        argv[1] = '-tt'
        execute(argv+[self.alias, cmd])
        self.connection.close()
        if not self.inspect()['docker_access']:
            raise SetupError('新的 SSH 登录仍不能访问 Docker，请根据上方 Docker／sudo 提示修复后重跑')

    def prepare_robot(self, bundle, manifest):
        self.step('准备机器人镜像和运行工具')
        self.request.update(image=manifest['image'], image_fingerprint=manifest['image_fingerprint'])
        info = self.inspect()
        if info['docker_installed'] and not info['docker_access']:
            self.admin(info, bundle='')
            info = self.inspect()
        runtime = info.get('runtime') or {}
        self.switch_image = False
        if runtime and (runtime['image'] != manifest['image'] or not info.get('selected_image_matches', False)):
            print('当前镜像：'+runtime['image']+'；安装包镜像：'+manifest['image'])
            if runtime['image'] == manifest['image']:
                print('镜像标签相同，但内容指纹已更新。')
            if runtime.get('development', {}).get('enabled'):
                print('当前启用了开发工作空间；切换镜像会停用该挂载，但保留源码。')
            self.switch_image = confirm('是否切换到安装包镜像？选择否会保留当前版本')
        if runtime and not self.switch_image and info['selected_image_available']:
            print('当前部署已存在，保留实际镜像和开发工作空间。')
            return info
        if runtime and not self.switch_image and runtime['image'] != manifest['image']:
            raise SetupError('当前选择的镜像已缺失；可重跑并确认使用安装包版本，或提供原版本部署包')
        needs_install = not runtime or self.switch_image or not info['image_matches']
        if (info['navigation_running'] or info.get('container_running') or info['busy']) and needs_install:
            raise SetupError('机器人有运行中的导航／采集；部署包已准备，未停止任何任务。正常结束后重跑向导即可')
        version = json.loads((bundle/'release.json').read_text()).get('version', RELEASE) if (bundle/'release.json').is_file() else manifest.get('version', RELEASE)
        if not re.fullmatch(r'v[0-9]+\.[0-9]+\.[0-9]+(?:-[A-Za-z0-9.-]+)?', version):
            raise SetupError('部署包版本格式无效')
        remote_bundle = self.identity['home']+'/pm01_navigation_deploy/releases/'+version+'-'+manifest['image_fingerprint'][:12]
        if not info['docker_installed']:
            self.step('为机器人补齐离线 Docker 安装包')
            inventory = self.connection.helper('inventory', self.request)
            stage = self.cache/'engine'/self.identity['machine_id']/manifest['image_fingerprint'][:12]
            stage.mkdir(parents=True, exist_ok=True)
            status = stage/'robot-status'
            status.write_bytes(base64.b64decode(inventory['status']))
            # Never modify the shared immutable release cache for one robot.
            private = stage/'bundle'
            prepare_private_bundle(bundle, private)
            from pm01_navigation_runtime.containers import bundle as container_bundle
            metadata = json.loads((private/'manifest.json').read_text())
            complete = metadata.get('engine_inventory_sha256') == hashlib.sha256(status.read_bytes()).hexdigest()
            complete = complete and any(name.startswith('engine-debs/') for name in metadata['files'])
            if not complete:
                shutil.rmtree(private/'engine-debs', ignore_errors=True)
                container_bundle.prepare_engine(status, private)
            bundle = private
        self.connection.call('mkdir -p '+shlex.quote(remote_bundle))
        self.step('传输已校验的机载部署包（中断后重跑可续传）')
        self.connection.copy(str(bundle)+'/', remote_bundle+'/')
        if not info['docker_access']:
            self.admin(info, bundle=remote_bundle)
        self.step('加载或复用机器人镜像')
        result = self.connection.helper('install', dict(self.request, bundle=remote_bundle,
                                                       switch_image=self.switch_image), timeout=2400)
        self.state['image'] = result['image']
        return self.inspect()

    def configure(self, info):
        self.step('生成两端配置')
        import yaml
        from pm01_navigation_runtime.deployment.configuration import apply_deployment
        deployment = self.folder/'deployment.yaml'
        old = yaml.safe_load(deployment.read_text()) if deployment.exists() else None
        # Preserve site tuning and GUI settings from any existing same-device path.
        if not old:
            for candidate in sorted((self.root/'robots').glob('*/deployment.yaml')):
                try:
                    value = yaml.safe_load(candidate.read_text())
                except (OSError, yaml.YAMLError):
                    continue
                if not isinstance(value, dict):
                    continue
                if value.get('robot', {}).get('machine_id') == self.identity['machine_id']:
                    old = value
                    break
        if not (self.folder/'console.yaml').exists():
            candidates = [self.root/'console.yaml', *(self.root/'robots').glob('*/console.yaml')]
            for candidate in candidates:
                if not candidate.is_file():
                    continue
                try:
                    value = yaml.safe_load(candidate.read_text())
                except (OSError, yaml.YAMLError):
                    continue
                if not isinstance(value, dict):
                    continue
                if value.get('expected_machine_id') == self.identity['machine_id']:
                    write_file(self.folder/'console.yaml', yaml.safe_dump(value, sort_keys=False, allow_unicode=True))
                    break
        value = old or dict(schema=1, robot={}, developer={}, network={})
        value['robot'].update(ssh_host=self.alias, address=self.address, machine_id=self.identity['machine_id'],
                              container_tool=self.request['container_tool'], container_config=self.request['container_config'])
        value['developer']['address'] = route_address(self.address)
        value['network'].setdefault('ros_domain_id', 69)
        value['network'].setdefault('robot_lan_address', '192.168.0.162')
        value['network'].setdefault('motion_peer', '192.168.0.163')
        write_file(deployment, yaml.safe_dump(value, sort_keys=False, allow_unicode=True))
        try:
            applied = apply_deployment(deployment)
        except RuntimeError as error:
            raise SetupError(str(error)) from error
        self.state.update(image=applied['robot']['image'], console=str(self.folder/'console.yaml'))
        self.deployment = deployment

    def clock(self):
        self.step('检查两端时钟')
        from pm01_navigation_runtime.system.clock_check import check
        result = check(self.alias, self.identity['machine_id'])
        self.state['clock'] = result
        if abs(result['robot_minus_developer_sec'])+result['uncertainty_sec'] <= .05:
            print('两端时钟误差上界在 50 ms 内，无需调整。')
            return
        print('机器人时间减开发机时间：{:.3f} 秒（测量不确定度 ±{:.3f} 秒）。'.format(
            result['robot_minus_developer_sec'], result['uncertainty_sec']))
        print('校时会调整机器人系统时间，可能向前或向后跳变；工具不停止、重启或修改现有服务。')
        if confirm('以开发机时间为准，确认现在校准机器人时间？'):
            from pm01_navigation_runtime.system.sync_clock import synchronise
            self.state['clock'] = synchronise(self.alias, self.identity['machine_id'])
        else:
            self.state['clock'].update(aligned=False, adjustment='user_skipped')
            self.warnings.append('本次未校时，机器人时间和现有服务保持不变；跨机消息年龄可能受时差影响')

    def probe(self, info):
        self.step('限时检查 ROS 双向通信')
        if info['navigation_running'] or info['busy']:
            self.state['communication'] = dict(state='not_checked', reason='existing_navigation_or_acquisition')
            self.warnings.append('现有导航／采集未停止，本轮未额外运行 ROS 通信采样')
            return
        nonce = uuid.uuid4().hex[:12]
        target = self.identity['home']+'/.cache/pm01_navigation/setup/probe-'+nonce+'.py'
        self.connection.call('mkdir -p '+shlex.quote(str(Path(target).parent)))
        self.connection.copy(self.workspace/'src/pm01_navigation_client/pm01_navigation_client/diagnostics/dds_probe.py', target)
        request = dict(self.request, nonce=nonce, probe_script=target)
        try:
            self.connection.helper('probe-start', request)
            shell = ('source '+shlex.quote(str(self.workspace/'scripts/env.sh'))+' --config '+
                     shlex.quote(str(self.folder/'console.yaml'))+' && exec /usr/bin/python3 '+
                     shlex.quote(str(self.workspace/'src/pm01_navigation_client/pm01_navigation_client/diagnostics/dds_probe.py'))+' client '+nonce)
            result = execute(['bash', '-c', shell], capture=True, check=False, timeout=25,
                             env=clean_build_environment())
            if result.returncode and not result.stdout.strip():
                raise SetupError('本机 ROS 测试节点未完成：'+(result.stderr.strip()[-1000:] or str(result.returncode)))
            value = json_result(result.stdout, require_ok=False)
            if result.returncode or not value.get('service_confirmed'):
                raise SetupError('ROS 双向消息／服务尚未确认')
            self.state['communication'] = dict(state='confirmed', **value)
            print('双向消息和测试服务均已收到；未启动传感器或导航。')
        except (SetupError, subprocess.TimeoutExpired) as error:
            self.state['communication'] = dict(state='unconfirmed', reason=str(error))
            self.warnings.append('ROS 通信尚未确认：'+str(error)+'。安装配置保留，不以此阻止环境加载')
        finally:
            try:
                self.connection.helper('probe-stop', request)
                self.connection.call('rm -f -- '+shlex.quote(target))
            except (SetupError, subprocess.TimeoutExpired) as error:
                self.warnings.append('测试清理回执未确认，测试节点最多运行 35 秒后退出：'+str(error))

    def run(self):
        self.desktop()
        self.connect()
        bundle, manifest = self.bundle()
        info = self.prepare_robot(bundle, manifest)
        self.configure(info)
        self.clock()
        self.probe(self.inspect())
        self.step('选择本次机器人环境')
        activate(self.root, self.workspace, self.folder/'console.yaml')
        self.state.update(navigation_started=False, motion_started=False, warnings=self.warnings)
        self.save('complete')
        print('\n准备完成。机器人：{}；实际镜像：{}'.format(self.identity['hostname'], self.state['image']))
        print('配置目录：'+str(self.folder))
        for warning in self.warnings:
            print('提示：'+warning)
        print('本向导未启动雷达、LIO、导航或运动输出。继续实操前运行：')
        print('source '+shlex.quote(str(self.root/'active.env')))
        return self.state


def main(workspace):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', help='Existing SSH alias or ubuntu@IPv4; otherwise prompted')
    download = parser.add_mutually_exclusive_group()
    download.add_argument('--bundle', help='Existing verified local ARM64 deployment directory')
    download.add_argument('--manifest-url', help='Custom deployment manifest; bypass source selection')
    download.add_argument('--download-source', choices=('aliyun', 'github'), help='Select and remember download source')
    parser.add_argument('--list', action='store_true', help='List recorded devices without connecting')
    args = parser.parse_args()
    if args.list:
        print(json.dumps(records(CONFIG), ensure_ascii=False, indent=2))
        return 0
    CONFIG.mkdir(parents=True, exist_ok=True)
    wizard = Wizard(args, workspace=workspace)
    with (CONFIG/'setup.lock').open('a') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            print('另一份准备向导正在运行；等待它结束后重试即可', file=sys.stderr)
            return 1
        try:
            wizard.run()
            return 0
        except (Exception, KeyboardInterrupt) as error:
            message = '用户中断，已完成内容保留' if isinstance(error, KeyboardInterrupt) else str(error)
            for token in (os.environ.get('PM01_DOWNLOAD_TOKEN'), os.environ.get('GITLAB_TOKEN')):
                if token:
                    message = message.replace(token, '[凭据已隐藏]')
            wizard.save('incomplete', message)
            print('\n未完成的步骤：'+wizard.stage+'\n'+message, file=sys.stderr)
            print('修正后重新运行 bash scripts/setup_hardware.sh；无需删除目录。', file=sys.stderr)
            return 1
        finally:
            if wizard.connection:
                try:
                    wizard.connection.close()
                except (SetupError, subprocess.TimeoutExpired):
                    pass


