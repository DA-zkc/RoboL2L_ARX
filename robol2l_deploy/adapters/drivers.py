"""Choose one robot library at process startup; no live backend switching."""
import math
from ..errors import ApiError,ErrorCode

BACKENDS = ('panda_py', 'franky')


def resolve_speed_factor(requested,default=.05,maximum=1.):
    value=default if requested is None else requested
    if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value) or not 0<value<=maximum:
        raise ApiError(ErrorCode.INVALID_REQUEST,'speed_factor_out_of_range',
            details={'requested':str(value),'allowed':'0 < speed_factor <= '+str(maximum),'default':default})
    return float(value)


def make_driver(backend, address, *, motion=None):
    settings=motion or {}
    if backend == 'panda_py':
        from .panda import PandaDriver
        driver=PandaDriver(address)
    elif backend == 'franky':
        from .franky import FrankyDriver
        driver=FrankyDriver(address)
    else:raise ValueError('unknown_robot_backend: '+str(backend))
    maximum=float(settings.get('max_speed_factor',1.))
    if not math.isfinite(maximum) or not 0<maximum<=1:raise ValueError('invalid_max_speed_factor')
    driver.speed_factor=resolve_speed_factor(settings.get('default_speed_factor',.05),maximum=maximum)
    driver.max_speed_factor=maximum
    return driver
