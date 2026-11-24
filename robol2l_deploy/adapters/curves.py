"""Public trajectory samples mapped to the installed native reference evaluators."""
import numpy as np
from scipy.interpolate import CubicSpline


def coefficients(positions, dt_s):
    points=np.asarray(positions,dtype=float)
    if points.ndim!=2 or points.shape[1]!=7 or not len(points) or not np.isfinite(points).all():
        raise ValueError('trajectory_requires_finite_seven_joint_samples')
    if not np.isfinite(dt_s) or dt_s<=0:raise ValueError('invalid_trajectory_dt')
    if len(points)==1:points=np.repeat(points,2,axis=0)
    spline=CubicSpline(np.arange(len(points))*dt_s,points,bc_type='clamped')
    return spline.c.transpose(1,0,2).tolist(),points


def sample_interval(interpolation_dt_s=None, maximum_trajectory_dt_s=None):
    values=[v for v in (interpolation_dt_s,maximum_trajectory_dt_s) if v is not None]
    if any(not np.isfinite(v) or v<=0 for v in values):raise ValueError('invalid_trajectory_dt')
    return min(values) if values else .02
