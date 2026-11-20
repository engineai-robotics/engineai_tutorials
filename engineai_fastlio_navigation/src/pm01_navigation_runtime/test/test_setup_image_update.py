import fcntl
import json
from types import SimpleNamespace

import pytest

from pm01_navigation_runtime.containers import manager
from pm01_navigation_runtime.deployment import setup_hardware, setup_remote


@pytest.fixture
def installation(tmp_path, monkeypatch):
    monkeypatch.setenv('HOME', str(tmp_path))
    monkeypatch.setattr(manager.os, 'getuid', lambda: 1000)
    monkeypatch.setattr(manager, 'machine', lambda: 'a'*32)
    monkeypatch.setattr(manager, 'architecture', lambda: 'arm64')
    monkeypatch.setattr(setup_remote, 'verify_identity', lambda _: None)
    monkeypatch.setattr(setup_remote, 'busy_processes', lambda: [])
    monkeypatch.setattr(setup_remote, 'load_runtime', lambda _: manager)
    config = tmp_path/'.config/pm01_navigation/container.json'
    config.parent.mkdir(parents=True)
    tag = 'pm01-navigation:v1.0.0-arm64'
    images = {}
    for name in ('old', 'new'):
        images[name] = dict(Id='sha256:'+name, Architecture='arm64', Os='linux',
                           RootFS=dict(Layers=['sha256:'+name]),
                           Config=dict(Labels={'cn.engineai.pm01.project': manager.PROJECT,
                                               'cn.engineai.pm01.profile': 'hardware'}))
    state = SimpleNamespace(tag_image='new', container=True, running=False,
                            calls=[], loads=0, config=config, tag=tag, images=images,
                            cid='b'*64, nav=False, busy=[], extra='', stops=[])
    initial = dict(schema=1, machine_id='a'*32, image=tag, image_id=images['old']['Id'],
                   container='pm01-navigation-test',
                   session_config=str(config.parent/'docker/session.json'),
                   development=dict(enabled=True, workspace=str(tmp_path/'development')))
    config.write_text(json.dumps(initial))
    owner = manager.Runtime(config).owner

    def docker(args, timeout=30):
        state.calls.append(args)
        if args[:3] == ['docker', 'image', 'inspect']:
            image = images[state.tag_image] if args[3] == tag else next(
                value for value in images.values() if value['Id'] == args[3])
            return json.dumps([image])
        if args[:3] == ['docker', 'container', 'ls']:
            return initial['container'] if state.container else ''
        if args[:3] == ['docker', 'container', 'inspect']:
            return json.dumps([dict(Id=state.cid, State=dict(Running=state.running, Pid=10 if state.running else 0,
                                         Status='running' if state.running else 'exited'),
                                   Config=dict(Labels={'cn.engineai.pm01.owner': owner,
                                                       'cn.engineai.pm01.project': manager.PROJECT}))])
        if args == ['docker', 'container', 'rm', initial['container']]:
            assert not state.running
            state.container = False
            return ''
        pytest.fail('Unexpected Docker operation: '+repr(args))

    monkeypatch.setattr(manager, 'run', docker)

    def remote_command(args, **kwargs):
        if args == ['docker', 'top', state.cid, '-eo', 'pid,ppid,comm,args']:
            return SimpleNamespace(stdout='PID PPID COMMAND COMMAND\n10 8 tini /usr/bin/tini -- sleep infinity\n'
                                   +'12 10 sleep sleep infinity\n'+state.extra)
        assert args == ['docker', 'container', 'stop', '--time', '2', state.cid]
        state.stops.append(state.cid)
        state.running = False

    monkeypatch.setattr(setup_remote, 'command', remote_command)
    manifest = dict(image=tag, architecture='arm64',
                    image_fingerprint=manager.image_fingerprint(images['new']))
    state.manifest = manifest
    monkeypatch.setattr(manager, 'verify_bundle', lambda _: manifest)

    def load(_):
        state.loads += 1
        state.tag_image = 'new'

    monkeypatch.setattr(manager, 'load_bundle', load)
    def inspect(_):
        info = dict(navigation_running=state.nav, container_running=state.running, busy=state.busy)
        if state.running and not state.nav and not state.busy:
            info['idle_container_id'], info['container_blocker'] = setup_remote.idle_container(manager.Runtime(config))
        return info

    monkeypatch.setattr(setup_remote, 'inspect', inspect)
    bundle = tmp_path/'bundle'
    bundle.mkdir()
    (bundle/'pm01_container.py').write_text('# container entry point\n')
    state.request = dict(machine_id='a'*32, container_config=str(config),
                         container_tool=str(tmp_path/'deploy/pm01_container.py'),
                         bundle=str(bundle), switch_image=True)
    state.initial = initial
    return state


@pytest.mark.parametrize('already_loaded', [True, False])
def test_same_tag_update_selects_new_image_and_removes_stopped_container(installation, already_loaded):
    state = installation
    state.tag_image = 'new' if already_loaded else 'old'
    result = setup_remote.install(state.request)
    selected = json.loads(state.config.read_text())
    assert selected['image'] == state.tag
    assert selected['image_id'] == state.images['new']['Id']
    assert selected['session_config'] == state.initial['session_config']
    assert selected['development'] == dict(state.initial['development'], enabled=False)
    assert not state.container
    assert state.loads == int(not already_loaded)
    assert result['ok'] and not result['navigation_started'] and not result['motion_started']


def test_unconfirmed_update_preserves_selected_image_and_development(installation):
    state = installation
    setup_remote.install(dict(state.request, switch_image=False))
    assert json.loads(state.config.read_text()) == state.initial
    assert state.container and state.loads == 0


def test_first_install_selects_bundle_image(installation):
    state = installation
    state.config.unlink()
    state.container = False
    setup_remote.install(dict(state.request, switch_image=False))
    assert json.loads(state.config.read_text())['image_id'] == state.images['new']['Id']
    assert state.loads == 0


def test_running_session_is_untouched(installation):
    state = installation
    state.running = state.nav = True
    with pytest.raises(RuntimeError, match='导航会话正在运行'):
        setup_remote.install(state.request)
    assert json.loads(state.config.read_text()) == state.initial
    assert not state.calls


def test_confirmed_switch_stops_only_the_inspected_idle_container(installation):
    state = installation
    state.running = True
    result = setup_remote.install(dict(state.request, idle_container_id=state.cid))
    assert result['stopped_idle_container'] and state.stops == [state.cid]
    assert not state.container and json.loads(state.config.read_text())['image_id'] == state.images['new']['Id']


@pytest.mark.parametrize('change,message', [
    ('replacement', '容器已变化'), ('shell', 'bash'), ('recording', 'ros2 bag record'),
    ('session_start', '执行容器操作'), ('no_switch', '已保留容器')])
def test_update_preserves_new_or_busy_work_after_transfer(installation, change, message):
    state = installation
    state.running = True
    req = dict(state.request, idle_container_id=state.cid)
    with state.config.with_suffix('.lock').open('a') as lock:
        if change == 'replacement':
            state.cid = 'c'*64
        elif change == 'shell':
            state.extra = '18 10 bash /bin/bash\n'
        elif change == 'recording':
            state.busy = [dict(names=['ros2 bag record'], pid=150)]
        elif change == 'session_start':
            fcntl.flock(lock, fcntl.LOCK_EX)
        else:
            req['switch_image'] = False
        with pytest.raises(RuntimeError, match=message):
            setup_remote.install(req)
    assert state.running and not state.stops and state.loads == 0
    assert json.loads(state.config.read_text()) == state.initial


def test_install_does_not_report_success_when_switch_leaves_old_content(installation, monkeypatch):
    monkeypatch.setattr(manager.Runtime, 'use_image', lambda *_: dict(ok=True))
    with pytest.raises(RuntimeError, match='Selected image content'):
        setup_remote.install(installation.request)


@pytest.mark.parametrize('confirm_update', [True, False])
def test_wizard_honors_same_tag_content_change(tmp_path, monkeypatch, confirm_update):
    wizard = setup_hardware.Wizard(SimpleNamespace(), root=tmp_path/'config',
                                  cache=tmp_path/'cache', workspace=tmp_path)
    tag = 'pm01-navigation:v1.0.0-arm64'
    manifest = dict(image=tag, image_fingerprint='a'*64, version='v1.0.0')
    wizard.identity = dict(home=str(tmp_path/'robot'))
    wizard.request = {}
    info = dict(runtime=dict(image=tag), docker_installed=True, docker_access=True,
                selected_image_matches=False, selected_image_available=True,
                image_matches=True, navigation_running=False, container_running=True, idle_container_id='b'*64, busy=[])
    monkeypatch.setattr(wizard, 'inspect', lambda: info)
    monkeypatch.setattr(setup_hardware, 'confirm', lambda _: confirm_update)
    calls = []

    def helper(action, request, **kwargs):
        calls.append((action, request))
        return dict(image=tag)

    wizard.connection = SimpleNamespace(call=lambda *_: None, copy=lambda *_: None, helper=helper)
    wizard.prepare_robot(tmp_path/'bundle', manifest)
    assert len(calls) == int(confirm_update)
    if confirm_update:
        assert calls[0][0] == 'install'
        assert calls[0][1]['switch_image'] is True
        assert calls[0][1]['idle_container_id'] == 'b'*64


def test_cleanup_rechecks_container_and_workspace_references(tmp_path, monkeypatch):
    monkeypatch.setenv('HOME', str(tmp_path))
    monkeypatch.setattr(manager, 'architecture', lambda: 'arm64')
    current, previous, container_image, dev_image, shared, unknown = [
        'sha256:'+c*64 for c in 'abcdef']
    refs = {container_image}
    removed = []
    config = tmp_path/'.config/pm01_navigation/container.json'
    config.parent.mkdir(parents=True)
    dev = tmp_path/'custom-workspace'
    dev.mkdir()
    (dev/'development.json').write_text(json.dumps({'initialized_image_id': dev_image}))
    config.write_text(json.dumps({'image_id': current, 'session_config': 'session.json',
                                  'development': {'workspaces': [str(dev)]}}))
    def docker(args, **kwargs):
        if args[1:3] == ['container', 'ls']:
            assert '-aq' in args
            data = 'container'
        elif args[1:3] == ['container', 'inspect']:
            data = '\n'.join(refs)
        elif args[1:3] == ['image', 'ls']:
            data = '\n'.join((current, previous, container_image, dev_image, shared, unknown))
        elif args[1:3] == ['image', 'inspect']:
            image = args[-1]
            data = json.dumps([{'Id': image, 'Architecture': 'arm64', 'Os': 'linux',
                                'RootFS': {'Layers': [image]}, 'RepoTags': ['other:latest'] if image == shared else [],
                                'Config': {'Labels': {'cn.engineai.pm01.profile': 'hardware',
                                                     'cn.engineai.pm01.project': manager.PROJECT}}}])
        else:
            assert args == ['docker', 'image', 'rm', '--no-prune', previous]
            removed.append(args[-1])
            data = ''
        return SimpleNamespace(stdout=data, returncode=0)
    monkeypatch.setattr(setup_remote, 'command', docker)
    request = dict(container_config=str(config), container_tool=str(tmp_path/'pm01_container.py'),
                   selected_id=current, previous_id=previous)
    assert [i['id'] for i in setup_remote.cleanup_images(request)['candidates']] == [previous]
    request['confirmed_ids'] = [previous, shared, dev_image]
    refs.add(previous)
    assert setup_remote.cleanup_images(request, remove=True)['removed'] == []
    refs.remove(previous)
    assert setup_remote.cleanup_images(request, remove=True)['removed'] == [previous]
    assert removed == [previous]


def test_cleanup_prompt_keeps_images_by_default(tmp_path, monkeypatch):
    wizard = setup_hardware.Wizard(SimpleNamespace(), root=tmp_path/'config',
                                  cache=tmp_path/'cache', workspace=tmp_path)
    calls = []
    def helper(action, request):
        calls.append(action)
        return {'candidates': [{'id': 'old', 'tags': [], 'previous': True}]}
    wizard.connection = SimpleNamespace(helper=helper)
    monkeypatch.setattr('builtins.input', lambda _: '')
    wizard.offer_image_cleanup({})
    assert calls == ['list-old-images']
