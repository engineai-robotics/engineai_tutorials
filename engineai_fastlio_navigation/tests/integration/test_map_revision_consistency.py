"""A globally rotated seed and its live layer are adopted as one snapshot."""
import math
import numpy as np
import pytest
from pm01_navigation_core.localization.slam_updates import SlamUpdate,SnapshotInbox
from pm01_navigation_core.mapping.continuous_occupancy import ContinuousOccupancy


@pytest.mark.parametrize('map_first',[True,False])
def test_loop_snapshot_preserves_clearing_across_rotation_and_origin_change(map_first):
    seed=np.zeros((60,60),dtype=np.int16);seed[29:32,39:42]=100
    grid=ContinuousOccupancy(seed,.1,(-3.,-3.,0.),ground_z=0.,max_range=8.)
    points=np.array([[x,y,0.] for x in np.arange(.5,1.7,.05) for y in np.arange(-.5,.6,.05)])
    for stamp in range(1,13):grid.integrate(points,(0.,0.,1.),stamp)
    old_live=grid.live
    inbox=SnapshotInbox()
    update=SlamUpdate.parse(dict(schema=1,backend='pm01-slam-2.6.10',epoch='fixture',
        revision=4,optimization_revision=4,stamp_ns=12_000_000_000,accepted=True,
        event='loop_closure',reason='loop_closure',anchor_id=0,
        pose=[1.,2.,math.pi/2],odom=[0.,0.,0.]))
    # Rotate/translate the complete raster, including its origin, without a new observation.
    snapshot=(seed.copy(),.1,(4.,-1.,math.pi/2))
    first,second=(lambda:inbox.map(update.stamp_ns,snapshot),lambda:inbox.update(update))
    if not map_first:first,second=second,first
    assert first() is None
    (data,resolution,origin),accepted=second()
    grid.rebase(data,resolution,origin,stamp=12.,map_from_odom=accepted.correction,
                optimized_anchors={accepted.anchor_id: accepted.pose})
    col,row=grid.cell(.95,3.05)
    assert grid.data[row,col]==0
    assert grid.live is not old_live and grid.frames==12
    assert grid.optimizations==1
