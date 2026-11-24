"""Typed JSON IPC. No pickle or executable payloads."""
import base64
from collections.abc import Mapping
from dataclasses import fields, is_dataclass
from enum import Enum
import numpy as np
from cap_harness import contracts
from cap_harness.errors import ApiError

TYPES = {name:value for name,value in vars(contracts).items() if isinstance(value,type) and is_dataclass(value)}
TYPES['ApiError'] = ApiError


def encode(value):
    if isinstance(value, Enum): return value.value
    if isinstance(value, np.ndarray):
        if value.dtype.kind not in 'biuf': raise ValueError('unsupported array')
        return {'$array':base64.b64encode(value.tobytes()).decode(), 'dtype':value.dtype.str, 'shape':list(value.shape)}
    if isinstance(value, np.generic): return value.item()
    if is_dataclass(value):
        if type(value).__name__ not in TYPES: raise ValueError('unsupported public type')
        return {'$type':type(value).__name__, 'fields':{f.name:encode(getattr(value,f.name)) for f in fields(value)}}
    if isinstance(value, Mapping): return {'$map':[[encode(k),encode(v)] for k,v in value.items()]}
    if isinstance(value, (tuple,list)): return [encode(v) for v in value]
    if value is None or isinstance(value,(bool,int,float,str)): return value
    raise ValueError('unsupported wire value')


def decode(value, depth=0):
    if isinstance(value,list): return [decode(v,depth+1) for v in value]
    if not isinstance(value,dict): return value
    if '$array' in value:
        dtype=np.dtype(value['dtype']);shape=value['shape']
        if dtype.kind not in 'biuf' or not isinstance(shape,list) or any(type(n) is not int or n<0 for n in shape):
            raise ValueError('invalid array')
        size=int(np.prod(shape,dtype=object))*dtype.itemsize
        raw=base64.b64decode(value['$array'],validate=True)
        if len(raw)!=size:raise ValueError('array size mismatch')
        result=np.frombuffer(raw,dtype=dtype).reshape(shape);result.setflags(write=False)
        return result
    if '$map' in value: return {decode(k,depth+1):decode(v,depth+1) for k,v in value['$map']}
    if '$type' in value:
        cls=TYPES.get(value['$type'])
        if cls is None:raise ValueError('unknown public type')
        return cls(**{k:decode(v,depth+1) for k,v in value['fields'].items()})
    raise ValueError('invalid typed envelope')
