"""Bounded A* and conservative VFH fallback on an observed local grid.

Reuse FieldMap.smooth; forbid nearest-free teleportation, unknown traversal,
diagonal corner cutting, and shortcuts through unchecked cells.
"""
import heapq
import math
import time
import numpy as np
from uav_planning.field_map import FieldMap
from uav_mapping.rolling_grid import ray_cells


class LocalGridPlanner(FieldMap):
    def __init__(self, occupancy, resolution, origin):
        self.occ = np.asarray(occupancy)
        self.ny, self.nx = self.occ.shape
        self.res = resolution
        self.X_MIN, self.Y_MIN = origin
        # FieldMap uses [x,y]; unknown must be blocked as well.
        self.grid = (self.occ != 0).T

    def _x2i(self, x): return math.floor((x - self.X_MIN) / self.res)
    def _y2j(self, y): return math.floor((y - self.Y_MIN) / self.res)

    def line_free(self, p0, p1):
        a, b = (self._x2i(p0[0]), self._y2j(p0[1])), (self._x2i(p1[0]), self._y2j(p1[1]))
        return all(self.in_bounds(x, y) and not self.grid[x, y] for x, y in ray_cells(a, b))

    def plan(self, start, goal, budget=.025, max_expansions=6000):
        """Return (status, path). PARTIAL ends in reachable observed space;
        no claim of global completeness beyond the rolling map / camera FOV.
        """
        if self.occupied(*start): return 'BLOCKED_START', []
        a = self._x2i(start[0]), self._y2j(start[1])
        b = self._x2i(goal[0]), self._y2j(goal[1])
        h = lambda cell: math.hypot(cell[0]-b[0], cell[1]-b[1])
        queue, costs, came, closed = [(h(a), 0., a)], {a: 0.}, {}, set()
        best = a
        deadline = time.perf_counter() + budget
        reached = False
        while queue:
            if len(closed) >= max_expansions or time.perf_counter() > deadline:
                return 'BUDGET', []
            _, cost, cell = heapq.heappop(queue)
            if cell in closed: continue
            closed.add(cell)
            if h(cell) < h(best): best = cell
            if cell == b:
                best, reached = cell, True
                break
            x, y = cell
            for dx, dy in ((1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)):
                nxt = x+dx, y+dy
                if not self.in_bounds(*nxt) or self.grid[nxt]: continue
                if dx and dy and (self.grid[x+dx, y] or self.grid[x, y+dy]): continue
                nc = cost + math.hypot(dx, dy)
                if nc < costs.get(nxt, math.inf):
                    costs[nxt], came[nxt] = nc, cell
                    heapq.heappush(queue, (nc+h(nxt), nc, nxt))
        if not reached and (h(a)-h(best))*self.res < .3:
            return 'NO_PATH', []
        cells = [best]
        while cells[-1] in came: cells.append(came[cells[-1]])
        cells.reverse()
        path = [start] + [(self._i2x(x), self._j2y(y)) for x,y in cells[1:]]
        if reached:
            if not self.line_free(path[-1], goal): return 'NO_PATH', []
            path.append(goal)
        path = self.smooth(path)
        if not all(self.line_free(a, b) for a,b in zip(path, path[1:])):
            return 'NO_PATH', []
        return ('ASTAR' if reached else 'PARTIAL'), path

    def vfh(self, start, goal, distance=.4, clearance=.8, previous=None):
        """72-sector binary polar histogram of checked straight corridors.
        Unlike the old point-only VFH, unknown sectors are BLOCKED. This is
        a bounded fallback, not a substitute for A* after an actual NO_PATH.
        """
        target = math.atan2(goal[1]-start[1], goal[0]-start[0])
        wrap = lambda a: (a+math.pi) % (2*math.pi)-math.pi
        candidates = []
        for k in range(72):
            angle = 2*math.pi*k/72
            end = (start[0]+clearance*math.cos(angle), start[1]+clearance*math.sin(angle))
            if not self.line_free(start, end): continue
            cost = abs(wrap(angle-target))
            if previous is not None: cost += .3*abs(wrap(angle-previous))
            candidates.append((cost, angle))
        if not candidates: return None
        _, angle = min(candidates)
        return (start[0]+distance*math.cos(angle), start[1]+distance*math.sin(angle))


def bounded_step(start, path, distance):
    for point in path[1:]:
        delta = np.array(point)-start
        length = np.linalg.norm(delta)
        if length > 1e-6:
            return tuple(np.asarray(start)+delta*min(1., distance/length))
    return tuple(start)
