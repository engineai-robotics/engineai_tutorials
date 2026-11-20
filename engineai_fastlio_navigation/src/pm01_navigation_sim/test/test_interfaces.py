"""Simulation remains a sensor/actuator adapter without extra odometry aiding."""
from pathlib import Path


def test_platform_has_no_retired_aiding_entrypoint():
    root = Path(__file__).resolve().parents[1]
    assert not (root/'scripts/pm01_leg_velocity.py').exists()
    assert not (root/'pm01_navigation_sim/estimation').exists()
    adapter = (root/'src/motion/sim_cmd_vel_adapter.cpp').read_text()
    assert '/cmd_vel' in adapter and '/pm01/walking_cmd_vel' in adapter
