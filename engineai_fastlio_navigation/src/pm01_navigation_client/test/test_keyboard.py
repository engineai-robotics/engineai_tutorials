import argparse
import pytest

from pm01_navigation_client.keyboard import KeyboardState, speed


@pytest.mark.parametrize('key,expected', [('w', (.4, 0.)), ('s', (-.4, 0.)),
                                          ('a', (0., .7)), ('d', (0., -.7))])
def test_hold_and_release(key, expected):
    state = KeyboardState()
    state.press(key)
    assert state.velocity() == expected
    state.release(key)
    assert state.velocity() == (0., 0.)


def test_space_never_resumes_an_already_held_key():
    state = KeyboardState()
    state.press('w')
    state.press('space')
    state.release('space')
    state.press('w')  # A repeat cannot restart the held key.
    state.press('a')
    assert state.velocity() == (0., 0.)
    state.release('w')
    state.release('a')
    state.press('w')
    assert state.velocity() == (.4, 0.)


def test_space_held_blocks_new_keys():
    state = KeyboardState()
    state.press('space')
    state.press('w')
    state.release('space')
    assert state.velocity() == (0., 0.)
    state.release('w')
    state.press('d')
    assert state.velocity() == (0., -.7)


def test_opposites_and_simultaneous_turn():
    state = KeyboardState(.15, .3)
    for key in 'wasd':
        state.press(key)
    assert state.velocity() == (0., 0.)
    state.release('s')
    state.release('d')
    assert state.velocity() == (.15, .3)


def test_focus_loss_requires_fresh_key_press():
    state = KeyboardState()
    state.press('w')
    state.reset()
    assert not state.held
    assert state.velocity() == (0., 0.)
    state.press('s')
    assert state.velocity() == (-.4, 0.)


@pytest.mark.parametrize('value', ['0', '-1', 'nan', 'inf', '.71'])
def test_invalid_speed(value):
    with pytest.raises(argparse.ArgumentTypeError):
        speed(value)
