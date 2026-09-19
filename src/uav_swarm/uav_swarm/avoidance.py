"""ROS-independent common-frame transforms and lightweight velocity avoidance."""
import math
import numpy as np


def rotate_xy(vector, yaw):
    vector = np.asarray(vector, dtype=float)
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([c*vector[0]-s*vector[1], s*vector[0]+c*vector[1]])


def local_to_common(position, offset, yaw):
    position = np.asarray(position, dtype=float)
    out = position.copy()
    out[:2] = rotate_xy(position[:2], yaw) + np.asarray(offset, dtype=float)
    return out


def common_to_local(position, offset, yaw):
    position = np.asarray(position, dtype=float)
    out = position.copy()
    out[:2] = rotate_xy(position[:2]-np.asarray(offset, dtype=float), -yaw)
    return out


def closest_approach(relative_position, relative_velocity, horizon):
    """Return time and separation at closest approach in [0, horizon]."""
    p = np.asarray(relative_position, dtype=float)
    v = np.asarray(relative_velocity, dtype=float)
    vv = float(v @ v)
    t = 0.0 if vv < 1e-9 else float(np.clip(-(p @ v)/vv, 0.0, horizon))
    return t, float(np.linalg.norm(p+v*t))


def candidate_velocities(preferred, max_speed, rings=4, sectors=32):
    preferred = np.asarray(preferred, dtype=float)
    speed = np.linalg.norm(preferred)
    result = [preferred if speed <= max_speed else preferred*max_speed/speed, np.zeros(2)]
    base = math.atan2(preferred[1], preferred[0]) if speed > 1e-6 else 0.0
    # Alternating angle order gives a deterministic right-hand tie break.
    offsets = [0.0]
    for i in range(1, sectors//2+1):
        offsets.extend((-2*math.pi*i/sectors, 2*math.pi*i/sectors))
    for ring in range(1, rings+1):
        magnitude = max_speed*ring/rings
        for angle in offsets[:sectors]:
            result.append(magnitude*np.array([math.cos(base+angle), math.sin(base+angle)]))
    return result


def select_safe_velocity(position, velocity, preferred, neighbors, max_speed=.6,
                         horizon=2.0, base_radius=.65, delay_margin=.25,
                         own_id=1):
    """Select the closest safe sampled velocity using communicated neighbor intent.

    neighbors entries contain position, velocity, age, variance and uav_id. Higher
    IDs yield slightly more, removing symmetric head-on choices deterministically.
    """
    full_position = np.asarray(position, dtype=float)
    position = full_position[:2]
    velocity = np.asarray(velocity, dtype=float)[:2]
    preferred = np.asarray(preferred, dtype=float)[:2]
    scored = []
    for candidate in candidate_velocities(preferred, max_speed):
        feasible, minimum = True, math.inf
        for n in neighbors:
            neighbor_position = np.asarray(n['position'], dtype=float)
            p = neighbor_position[:2]-position
            nv = np.asarray(n['velocity'], dtype=float)[:2]
            age = max(0.0, float(n.get('age', 0.0)))
            variance = max(0.0, float(n.get('variance', 0.0)))
            rel_speed = np.linalg.norm(candidate-nv)
            radius_3d = base_radius + 2.0*math.sqrt(variance) + rel_speed*(age+delay_margin)
            # Higher id yields more in a symmetric encounter, but nobody ignores a collision.
            if int(own_id) > int(n.get('uav_id', own_id)):
                radius_3d += .08
            vertical = abs(neighbor_position[2]-full_position[2]) if len(neighbor_position)>2 and len(full_position)>2 else 0.
            if vertical >= radius_3d:continue
            radius = math.sqrt(max(0.,radius_3d*radius_3d-vertical*vertical))
            _, separation = closest_approach(p, nv-candidate, horizon)
            minimum = min(minimum, separation-radius)
            if separation < radius:
                feasible = False
                break
        change = float(np.sum((candidate-preferred)**2))
        # Prefer slowing over a large sideways excursion; small cross bias chooses right.
        side = float(preferred[0]*candidate[1]-preferred[1]*candidate[0])
        score = change + .02*np.linalg.norm(candidate) - 1e-4*side
        scored.append((not feasible, score, -minimum, candidate))
    feasible = [item for item in scored if not item[0]]
    if feasible:
        chosen=min(feasible,key=lambda item:(item[1],item[2]))
        return chosen[3],True,-chosen[2]
    # If the sampled set has no safe velocity, maximize clearance before comfort.
    chosen=min(scored,key=lambda item:(item[2],item[1]))
    return chosen[3],False,-chosen[2]
