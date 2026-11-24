"""Installed device adapters; importing this package opens no devices."""

from .drivers import BACKENDS

STATION_BACKENDS = (*BACKENDS, 'arx_x5_2023', 'arx_x5_2023_dual')


def make_station_adapter(station, backend, *, allow_connection=False, allow_motion=False):
    if backend == 'arx_x5_2023_dual':
        from .arx_dual import DualARXStation
        return DualARXStation.from_file(station, allow_connection=allow_connection, allow_motion=allow_motion)
    if backend == 'arx_x5_2023':
        from .arx import ARXStationAdapter

        return ARXStationAdapter.from_file(station, allow_connection=allow_connection,
                                           allow_motion=allow_motion)
    if backend not in BACKENDS:
        raise ValueError('unknown_robot_backend: ' + str(backend))

    from franka_cap.sdk import CapSDK
    from .fr3 import FR3Adapter

    return FR3Adapter(CapSDK(station), backend=backend)
