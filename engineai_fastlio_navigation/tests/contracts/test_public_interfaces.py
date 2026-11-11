from pathlib import Path
import ast

ROOT=Path(__file__).resolve().parents[2]


def test_documented_client_commands_keep_their_installed_names():
    tree=ast.parse((ROOT/'src/pm01_navigation_client/setup.py').read_text())
    call=next(n for n in ast.walk(tree) if isinstance(n,ast.Call) and isinstance(n.func,ast.Name) and n.func.id=='setup')
    entries=ast.literal_eval(next(k.value for k in call.keywords if k.arg=='entry_points'))['console_scripts']
    assert {e.split('=')[0].strip() for e in entries} == {'pm01_nav','pm01_sim_keyboard','pm01_clock_check','pm01_nav_debug'}
    from pm01_navigation_client import Navigator
    assert all(callable(getattr(Navigator,name)) for name in ('save_map','enable_motion','release_motion','relocalize','wait','poll'))


def test_original_shell_and_ros_launch_entrypoints_are_present():
    for name in ('setup_hardware.sh','install_dependencies.sh','build.sh','env.sh','pm01_container.py'):
        assert (ROOT/'scripts'/name).is_file()
    assert (ROOT/'src/pm01_navigation_bringup/launch/navigation.launch.py').is_file()
