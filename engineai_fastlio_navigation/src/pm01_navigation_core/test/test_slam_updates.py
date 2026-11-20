from pathlib import Path
import sys
import math
import pytest

from pm01_navigation_core.localization.slam_updates import SlamUpdate, SnapshotInbox, MapEventInbox


def update(revision=1, stamp=10, epoch='one', **kwargs):
    return SlamUpdate.parse(dict(schema=1, backend='pm01-slam-2.6.10', epoch=epoch,
        revision=revision, stamp_ns=stamp, accepted=True, event='accepted', reason='accepted',
        pose=[2., 3., math.pi/2], odom=[1., 0., 0.], **kwargs))


@pytest.mark.parametrize('map_first', [True, False])
def test_exact_join_is_independent_of_arrival_order(map_first):
    inbox = SnapshotInbox()
    first, second = (lambda: inbox.map(10, 'map'), lambda: inbox.update(update()))
    if not map_first: first, second = second, first
    assert first() is None
    grid, match = second()
    assert grid == 'map' and match.correction == pytest.approx((2., 2., 0., math.pi/2))
    assert inbox.map(10, 'duplicate') is None
    assert inbox.update(update()) is None


def test_newest_complete_pair_wins_without_using_a_later_unpaired_correction():
    inbox = SnapshotInbox()
    inbox.update(update(3, 30)); inbox.update(update(2, 20))
    assert inbox.map(20, 'second')[1].revision == 2
    assert inbox.map(30, 'third')[1].revision == 3
    inbox.update(update(1, 10))
    assert inbox.map(10, 'old') is None


def test_epoch_change_requires_restart_and_queue_is_bounded():
    inbox = SnapshotInbox(4)
    inbox.update(update()); inbox.map(10, 'map')
    with pytest.raises(ValueError, match='epoch'):
        inbox.update(update(2, 20, epoch='two'))
    inbox.map(30, 'same_session')
    assert inbox.update(update(3,30))[0]=='same_session'
    inbox = SnapshotInbox(4)
    for n in range(20): inbox.map(n, n)
    assert len(inbox.maps) == 4


def test_health_backlog_cannot_evict_map_revision_or_anchors():
    inbox = MapEventInbox()
    for kind in ('seed', 'revision', 'anchors'):
        assert inbox.append((kind, kind, 1.)) == 0
    dropped = sum(inbox.append(('/lio/status', n, 2.+n)) for n in range(1000))
    assert dropped == 992
    health = inbox.drain({'/lio/status'})
    assert [event[1] for event in health] == list(range(992, 1000))
    assert [event[0] for event in inbox.drain()] == ['seed', 'revision', 'anchors']
    assert inbox.drain() == []
