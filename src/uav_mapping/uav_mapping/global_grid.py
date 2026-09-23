"""Fixed-size, ROS-independent map of cells actually observed locally.

This is a visualization layer. Unknown cells stay unknown and it never feeds
the local planner. The most recent observation replaces a previous value.
"""
import math
import numpy as np


class GlobalGrid:
    def __init__(self, size=60., resolution=.1):
        if not (math.isfinite(size) and math.isfinite(resolution) and
                size > 0 and resolution > 0):
            raise ValueError('invalid global map size or resolution')
        self.res = resolution
        self.n = math.ceil(size / resolution)
        self.origin = -(self.n // 2) * resolution
        self.data = np.full((self.n, self.n), -1, np.int8)

    def clear(self):
        self.data.fill(-1)

    def update(self, cells, origin_x, origin_y, resolution):
        cells = np.asarray(cells, dtype=np.int8)
        if cells.ndim != 2 or not np.isfinite([origin_x, origin_y, resolution]).all():
            raise ValueError('invalid local map')
        if not np.isclose(resolution, self.res, rtol=0, atol=1e-6):
            raise ValueError('local and global resolutions differ')
        x0 = round((origin_x - self.origin) / self.res)
        y0 = round((origin_y - self.origin) / self.res)
        if abs(self.origin + x0 * self.res - origin_x) > self.res * .01 or \
                abs(self.origin + y0 * self.res - origin_y) > self.res * .01:
            raise ValueError('local map origin is not grid aligned')
        h, w = cells.shape
        gx0, gy0 = max(0, x0), max(0, y0)
        gx1, gy1 = min(self.n, x0 + w), min(self.n, y0 + h)
        if gx0 >= gx1 or gy0 >= gy1:
            return False
        observed = cells[gy0-y0:gy1-y0, gx0-x0:gx1-x0]
        target = self.data[gy0:gy1, gx0:gx1]
        known = (observed == 0) | (observed == 100)
        target[known] = observed[known]
        return bool(np.any(known))
