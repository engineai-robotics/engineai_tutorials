"""Sensor geometry shared by navigation nodes."""
import numpy as np

def read_xyz(msg):
    fields={f.name:f for f in msg.fields}
    names, formats, offsets=[],[],[]
    for name in ('x','y','z'):
        f=fields[name]
        if f.datatype not in (7,8): raise ValueError('Expected floating XYZ fields')
        names.append(name); offsets.append(f.offset)
        formats.append(('>' if msg.is_bigendian else '<')+('f4' if f.datatype==7 else 'f8'))
    dtype=np.dtype({'names':names,'formats':formats,'offsets':offsets,'itemsize':msg.point_step})
    data=np.ndarray((msg.height,msg.width),dtype=dtype,buffer=msg.data,
                    strides=(msg.row_step,msg.point_step))
    return np.column_stack([data[k].ravel() for k in names])


def points_array(msg):
    return read_xyz(msg).astype(np.float32)
