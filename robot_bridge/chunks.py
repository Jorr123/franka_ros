"""Preserve joint waypoints using C2 quintic segments and bounded time scaling."""

import math

from .protocol import JOINTS, joint_vector, number


def _bezier(control, u):
    values = list(control)
    while len(values) > 1:
        values = [(1 - u) * a + u * b for a, b in zip(values, values[1:])]
    return values[0]


def timed_chunk(current, waypoints, period, velocity_limits, acceleration_limits):
    """Return dense (time, positions, velocities, accelerations) samples.

    Derivative control-point bounds enforce limits over the entire polynomial,
    not just at sample times. Position control points remain in each segment's
    endpoint interval. One global stretch preserves relative waypoint timing.
    """
    positions = [joint_vector(list(current), "start")] + [joint_vector(q, "waypoint") for q in waypoints]
    h = number(period, "period")
    if not 0.01 <= h <= 1.0 or not 2 <= len(waypoints) <= 32:
        raise ValueError("Expected 2..32 waypoints and a 0.01..1 second period")
    vmax = joint_vector(list(velocity_limits), "velocity limits")
    amax = joint_vector(list(acceleration_limits), "acceleration limits")
    if min(vmax + amax) <= 0:
        raise ValueError("Positive velocity and acceleration limits are required")
    slopes = [[0.0] * 7 for _ in positions]
    for i in range(1, len(positions) - 1):
        for j in range(7):
            left = (positions[i][j] - positions[i - 1][j]) / h
            right = (positions[i + 1][j] - positions[i][j]) / h
            slopes[i][j] = math.copysign(min(abs(left), abs(right)), left) if left * right > 0 else 0.0
    segments = []
    stretch = 1.0
    for i in range(len(positions) - 1):
        controls = []
        for j in range(7):
            p, q = positions[i][j], positions[i + 1][j]
            v, w = slopes[i][j] * h, slopes[i + 1][j] * h
            b = [p, p + v / 5, p + 2 * v / 5, q - 2 * w / 5, q - w / 5, q]
            d = [5 * (y - x) / h for x, y in zip(b, b[1:])]
            dd = [4 * (y - x) / h for x, y in zip(d, d[1:])]
            stretch = max(stretch, max(map(abs, d)) / vmax[j], math.sqrt(max(map(abs, dd)) / amax[j]))
            controls.append((b, d, dd))
        segments.append(controls)
    # Small numerical margin keeps derivative bounds below the configured caps.
    stretch *= 1.000001
    duration = h * stretch
    samples = []
    waypoint_indices = []
    for i, controls in enumerate(segments):
        divisions = max(4, math.ceil(duration / 0.05),
                        math.ceil(max(abs(v) * h for _, d, _ in controls for v in d) / 0.001))
        if len(samples) + divisions + 1 > 2048:
            raise ValueError("Chunk exceeds 2048 dense trajectory samples")
        for n in range(0 if i == 0 else 1, divisions + 1):
            u = n / divisions
            samples.append(((i + u) * duration,
                            [_bezier(b, u) for b, _, _ in controls],
                            [_bezier(d, u) / stretch for _, d, _ in controls],
                            [_bezier(dd, u) / stretch ** 2 for _, _, dd in controls]))
        waypoint_indices.append(len(samples) - 1)
    return samples, {"actions_per_chunk": len(waypoints), "nominal_action_period_seconds": h,
                     "time_scale": stretch, "effective_action_period_seconds": duration,
                     "waypoint_sample_indices": waypoint_indices}
