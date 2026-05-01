import numpy as np
import numpy as np
from dataclasses import dataclass
from types import SimpleNamespace

import control

@dataclass
class PurePursuitParams:
    ol_model: SimpleNamespace
    lookahead_distance: float
    pruning_distance: float
    wheelbase: float
    desired_vel: float
    mode: str = "pwm"  # "velocity" or "acceleration"
    Kp_v: float = 1.0
    Kp_theta: float = 1.0

def pure_pursuit(waypoints, current_state, params: PurePursuitParams, logger=None):
    """Follow a series of waypoints, looking ahead to a specified distance.
    Waypoints close to the vehicle are considered "visited" and suggested for pruning.
    If there are no waypoints left to visit, stop moving.
    """
    if not len(waypoints):
        # Stop moving if no waypoints are provided
        match params.mode:
            case "velocity":
                control_input = [0, 0]
            case "acceleration":
                control_input = [-current_state[3], 0]
            case "pwm":
                control_input = [1500, 1500]
            case _:
                raise ValueError(f"Invalid control mode: {params.mode}")
        return control_input, current_state[:3], 0

    lookahead_point = []
    wp_idx = -1
    wp_prune_idx = 0
    min_dist = float('inf')
    min_dist_idx = -1
    for i in range(len(waypoints)):
        wp = waypoints[i]

        # if(not node is None):
        #     node.get_logger().warn(f"{type(wp[0])}, {type(current_state[0])}")

        vec_to_wp = np.array([wp[0] - current_state[0], wp[1] - current_state[1]])
        dist = np.linalg.norm(vec_to_wp)

        if dist < params.pruning_distance:
            wp_prune_idx = i + 1
        if dist < params.lookahead_distance:
            wp_idx = i
        elif wp_idx != -1:
            break
        elif dist < min_dist:
            min_dist = dist
            min_dist_idx = i
    if wp_idx == -1:
        lookahead_point = waypoints[min_dist_idx]
    elif wp_idx < len(waypoints)-1:
        lookahead_point = max_intersect_segment_circle(waypoints[wp_idx], waypoints[wp_idx + 1], current_state[:2], params.lookahead_distance)
    else:
        lookahead_point = waypoints[-1]
    
    # Compute the control input to steer towards the lookahead point
    dx = lookahead_point[0] - current_state[0]
    dy = lookahead_point[1] - current_state[1]
    heading_to_lp = np.arctan2(dy, dx)

    curvature = 2 * (np.sin(heading_to_lp - current_state[2]))/params.lookahead_distance

    control_input = [0, 0]
    match params.mode:
        case "velocity":
            turn_rate = curvature * current_state[3]
            control_input = [params.desired_vel, turn_rate]
        case "acceleration":
            vel_input = params.desired_vel - current_state[3]
            delta = np.arctan(curvature * params.wheelbase)
            control_input = [vel_input, delta]
        case "pwm":
            vel_input = params.desired_vel
            turn_radius = 1/curvature if curvature != 0 else 99999
            control_input = [1395, control.get_pwm_steer_from_turn_radius(params.ol_model, turn_radius)]
        case _:
            raise ValueError(f"Invalid control mode: {params.mode}")

    return control_input, lookahead_point, wp_prune_idx


def max_intersect_segment_circle(p1full, p2full, center, radius):
    """Analytically solve for where a line segment intersects a circle.
    If there are 2 intersections, take the one closest to the 2nd endpoint (p2full).
    """
    p1 = np.array(p1full)[:2]
    p2 = np.array(p2full)[:2]
    center = np.array(center)[:2]
    if np.linalg.norm(p1 - center) >= radius:
        return p1full
    if np.linalg.norm(p2 - center) <= radius:
        return p2full
    
    # Direction vector of the segment
    dfull = p2full - p1full
    d = p2 - p1
    # Vector from center to p1
    f = p1 - center
    
    # Quadratic coefficients: at^2 + bt + c = 0
    a = np.dot(d, d)
    b = 2 * np.dot(f, d)
    c = np.dot(f, f) - radius**2
    
    discriminant = b**2 - 4*a*c

    # Potential t values for the full line
    discriminant = np.sqrt(discriminant)
    t1 = (-b - discriminant) / (2 * a)
    t2 = (-b + discriminant) / (2 * a)
    
    points = []
    for t in [t1, t2]:
        if 0 <= t <= 1:
            points.append(p1full + t * dfull)
    return min(points, key=lambda p: np.linalg.norm(p[:2] - p2)) if points else []
