"""ROS-independent, bounded rolling obstacle grid. Arrays indexed [y, x].

Map axes stay fixed in local NWU; only the integer-cell origin rolls.
Published cells are binary: retained, inflated cloud hits are 100; all
other cells are 0. Each scan votes once per cell, with hits taking priority.
"""
import math
import numpy as np


def ray_cells(a, b):
    """Supercover grid traversal, including both cells at a diagonal corner."""
    x, y = a
    ex, ey = b
    dx, dy = ex - x, ey - y
    nx, ny = abs(dx), abs(dy)
    sx, sy = (1 if dx > 0 else -1), (1 if dy > 0 else -1)
    ix = iy = 0
    yield x, y
    while ix < nx or iy < ny:
        lhs, rhs = (1 + 2 * ix) * ny, (1 + 2 * iy) * nx
        if lhs == rhs:
            yield x + sx, y
            yield x, y + sy
            x, y, ix, iy = x + sx, y + sy, ix + 1, iy + 1
        elif lhs < rhs:
            x, ix = x + sx, ix + 1
        else:
            y, iy = y + sy, iy + 1
        yield x, y


def body_to_nwu(q):
    """PX4 Hamilton wxyz (body FRD -> NED) to body FLU -> NWU."""
    q = np.asarray(q, dtype=float)
    norm = np.linalg.norm(q)
    if not np.all(np.isfinite(q)) or not .9 < norm < 1.1:
        raise ValueError('invalid attitude quaternion')
    w, x, y, z = q / norm
    r = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                  [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                  [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
    flip = np.diag([1., -1., -1.])
    return flip @ r @ flip


def body_quaternion_to_nwu(q):
    """PX4 FRD-to-NED wxyz quaternion as FLU-to-NWU wxyz quaternion."""
    q = np.asarray(q, dtype=float)
    norm = np.linalg.norm(q)
    if not np.all(np.isfinite(q)) or not .9 < norm < 1.1:
        raise ValueError('invalid attitude quaternion')
    w, x, y, z = q / norm
    return np.array([w, x, -y, -z])


def slab_endpoint(sensor, point, altitude, half_height):
    """Clip a valid ray to the flight slab; no-return pixels never reach here.
    Returns (endpoint, hit). Rays starting outside the slab do not clear cells.
    """
    if abs(point[2]-altitude)<=half_height:return point,True
    if abs(sensor[2]-altitude)>half_height:return None,False
    boundary=altitude+(half_height if point[2]>altitude else -half_height)
    fraction=(boundary-sensor[2])/(point[2]-sensor[2])
    return sensor+fraction*(point-sensor),False


class RollingGrid:
    def __init__(self, size=12., resolution=.1, memory=8., inflation=.35):
        if not (resolution > 0 and size > 2 * resolution and memory > 0 and inflation >= 0):
            raise ValueError('invalid map dimensions/time/radius')
        self.res = resolution
        self.n = math.ceil(size / resolution)
        self.memory, self.inflation = memory, inflation
        self.origin = np.array([-self.n // 2, -self.n // 2], dtype=int)
        self.odds = np.zeros((self.n, self.n), np.float32)
        self.seen = np.full((self.n, self.n), -np.inf)

    def clear(self):
        self.odds.fill(0)
        self.seen.fill(-np.inf)

    def recenter(self, xy):
        new = np.floor(np.asarray(xy) / self.res).astype(int) - self.n // 2
        dx, dy = new - self.origin
        if abs(dx) >= self.n or abs(dy) >= self.n:
            self.clear()
        elif dx or dy:
            for array, fill in ((self.odds, 0), (self.seen, -np.inf)):
                array[:] = np.roll(array, (-dy, -dx), axis=(0, 1))
                if dx > 0: array[:, -dx:] = fill
                if dx < 0: array[:, :-dx] = fill
                if dy > 0: array[-dy:, :] = fill
                if dy < 0: array[:-dy, :] = fill
        self.origin = new

    def cell(self, xy):
        return tuple(np.floor(np.asarray(xy) / self.res).astype(int) - self.origin)

    def inside(self, cell):
        return 0 <= cell[0] < self.n and 0 <= cell[1] < self.n

    def update(self, sensor, endpoints, now, altitude, half_height=.25):
        """Conservative height slab: hits in slab; clip floor/ceiling rays to
        the slab. Clipped rays can clear old hits but cannot erase new hits.
        Invalid/no-return pixels produce NO ray.
        """
        stale = now - self.seen > self.memory
        self.odds[stale], self.seen[stale] = 0, -np.inf
        start = self.cell(sensor[:2])
        if not self.inside(start):
            return
        hits, free, weak_free = set(), set(), set()
        for p in endpoints:
            if not np.all(np.isfinite(p)):continue
            endpoint,hit=slab_endpoint(sensor,p,altitude,half_height)
            if endpoint is None:continue
            end = self.cell(endpoint[:2])
            if hit and self.inside(end):
                hits.add(end)
            if abs(sensor[2] - altitude) <= half_height:
                for cell in ray_cells(start, end):
                    if not self.inside(cell): break
                    if cell != end:
                        (free if hit else weak_free).add(cell)
        free |= {c for c in weak_free if self.odds[c[1],c[0]]<=0}
        for x, y in free - hits:
            self.odds[y, x] = max(-2., self.odds[y, x] - .7)
            self.seen[y, x] = now
        for x, y in hits:
            self.odds[y, x] = min(3., self.odds[y, x] + 1.4)
            self.seen[y, x] = now

    def occupancy(self, now, inflate=True):
        recent = (now - self.seen >= 0) & (now - self.seen <= self.memory)
        out = np.zeros(self.odds.shape, np.int8)
        hit = recent & (self.odds >= 0)
        if not inflate:
            out[hit] = 100
            return out
        # Include half a cell diagonal: cells represent finite squares.
        radius = math.ceil(self.inflation / self.res + math.sqrt(.5))
        expanded = hit.copy()
        for dy in range(-radius, radius + 1):
            for dx in range(-radius, radius + 1):
                if math.hypot(dx, dy) > self.inflation / self.res + math.sqrt(.5): continue
                ys, ye = max(0, -dy), min(self.n, self.n - dy)
                xs, xe = max(0, -dx), min(self.n, self.n - dx)
                expanded[ys+dy:ye+dy, xs+dx:xe+dx] |= hit[ys:ye, xs:xe]
        out[expanded] = 100
        return out
