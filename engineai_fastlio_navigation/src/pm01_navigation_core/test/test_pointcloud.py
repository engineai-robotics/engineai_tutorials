from types import SimpleNamespace
import numpy as np
import pytest
from pm01_navigation_core.pointcloud import points_array,read_xyz
from pm01_navigation_core.geometry import rotation_xyzw


def test_big_endian_double_cloud_keeps_organized_row_padding():
    data=bytearray(64)
    for row,values in enumerate(((1.25,2.5,3.75),(-1.,-2.,-3.))):
        data[row*32:row*32+24]=np.asarray(values,dtype='>f8').tobytes()
    msg=SimpleNamespace(fields=[SimpleNamespace(name=n,datatype=8,offset=i*8) for i,n in enumerate('xyz')],
                        data=data,height=2,width=1,row_step=32,point_step=24,is_bigendian=True)
    expected=np.array([[1.25,2.5,3.75],[-1.,-2.,-3.]])
    np.testing.assert_array_equal(read_xyz(msg),expected)
    np.testing.assert_array_equal(points_array(msg),expected.astype(np.float32))
    assert read_xyz(msg).dtype == np.float64


def test_invalid_rotation_is_rejected_and_valid_quaternion_is_normalized():
    for q in ((0,0,0,0),(0,0,0,float('nan'))):
        with pytest.raises(ValueError):rotation_xyzw(*q)
    np.testing.assert_allclose(rotation_xyzw(0,0,0,2),np.eye(3))
