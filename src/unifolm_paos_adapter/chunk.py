"""Immutable bounded model response with a declared sample-time grid."""
from bisect import bisect_right
from dataclasses import dataclass
from types import MappingProxyType
import math

import numpy as np

FIELD_DIMS={'action.left_ee_rpy':6,'action.right_ee_rpy':6,
            'action.left_gripper':1,'action.right_gripper':1,
            'action.lower_body':15,'action.base_command':4,'action.pivot':7}


@dataclass(frozen=True)
class ModelActionChunk:
    actions: object
    received_at: float
    period_s: float
    horizon: int
    grid: tuple

    @classmethod
    def from_response(cls,response,*,received_at,period_s,expected_horizon=30):
        if (isinstance(received_at,bool)or not math.isfinite(received_at)or received_at<0 or
                isinstance(period_s,bool)or not math.isfinite(period_s)or period_s<=0 or
                type(expected_horizon)is not int or not 0<expected_horizon<=512):
            raise ValueError('invalid chunk time/grid configuration')
        if not isinstance(response,dict)or set(response)!=set(FIELD_DIMS):raise ValueError('official seven action fields required')
        arrays={}
        for name,dim in FIELD_DIMS.items():
            value=np.asarray(response[name])
            if value.shape!=(1,expected_horizon,dim)or value.dtype.kind!='f'or not np.isfinite(value).all():
                raise ValueError(f'invalid shape/dtype/value for {name}')
            value=value.copy();value.flags.writeable=False;arrays[name]=value
        grid=tuple(i*period_s for i in range(expected_horizon+1))
        if not math.isfinite(grid[-1]):raise ValueError('chunk duration outside finite range')
        return cls(MappingProxyType(arrays),float(received_at),float(period_s),expected_horizon,grid)


def base_goal_at(chunk,*,elapsed_s):
    if isinstance(elapsed_s,bool)or not math.isfinite(elapsed_s)or elapsed_s<0:
        raise ValueError('nonnegative finite elapsed time required')
    index=bisect_right(chunk.grid,elapsed_s)-1
    if index>=chunk.horizon:return None
    return tuple(float(v)for v in chunk.actions['action.base_command'][0,index,:3])
