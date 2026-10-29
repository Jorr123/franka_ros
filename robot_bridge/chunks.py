"""Preserve joint waypoints using C2 quintic segments and bounded time scaling."""

import math

from .protocol import JOINTS, joint_vector, number


def _bezier(control, u):
    values = list(control)
    while len(values) > 1:
        values = [(1 - u) * a + u * b for a, b in zip(values, values[1:])]
    return values[0]


def _absolute_bezier_bound(control, depth=4):
    """Conservative bound from subdivided convex hulls, over the whole curve."""
    if depth == 0:
        return max(map(abs, control))
    levels = [list(control)]
    while len(levels[-1]) > 1:
        levels.append([(a + b) / 2 for a, b in zip(levels[-1], levels[-1][1:])])
    left = [level[0] for level in levels]
    right = [level[-1] for level in reversed(levels)]
    return max(_absolute_bezier_bound(left, depth - 1),
               _absolute_bezier_bound(right, depth - 1))


def _adaptive_segments(positions, period, vmax, amax):
    # Start with stop-to-stop quintic peak bounds. Interior slopes are shared
    # across adjacent segments, preserving C2 continuity at every target.
    durations = [max(period, *(max(1.875 * abs(b - a) / v,
                                  math.sqrt((10 / math.sqrt(3)) * abs(b - a) / acc))
                              for a, b, v, acc in zip(p, q, vmax, amax)))
                 for p, q in zip(positions, positions[1:])]
    for _ in range(32):
        slopes = [[0.0] * 7 for _ in positions]
        for i in range(1, len(positions) - 1):
            for j in range(7):
                left = (positions[i][j] - positions[i - 1][j]) / durations[i - 1]
                right = (positions[i + 1][j] - positions[i][j]) / durations[i]
                slopes[i][j] = math.copysign(min(abs(left), abs(right)), left) if left * right > 0 else 0.0
        segments, factors = [], []
        for i, h in enumerate(durations):
            controls, factor = [], 1.0
            for j in range(7):
                p, q = positions[i][j], positions[i + 1][j]
                v, w = slopes[i][j] * h, slopes[i + 1][j] * h
                b = [p, p + v / 5, p + 2 * v / 5, q - 2 * w / 5, q - w / 5, q]
                d = [5 * (y - x) / h for x, y in zip(b, b[1:])]
                dd = [4 * (y - x) / h for x, y in zip(d, d[1:])]
                factor = max(factor, _absolute_bezier_bound(d) / vmax[j],
                             math.sqrt(_absolute_bezier_bound(dd) / amax[j]))
                controls.append((b, d, dd))
            segments.append(controls)
            factors.append(factor)
        if max(factors) <= 1.0:
            # One final uniform numerical margin preserves matching endpoint
            # derivatives. Each curve has already passed continuous bounds.
            return durations, segments
        durations = [h * factor * 1.000001 for h, factor in zip(durations, factors)]
    raise ValueError("Adaptive chunk timing did not converge within motion limits")


def _sample_adaptive(positions, period, vmax, amax):
    durations, segments = _adaptive_segments(positions, period, vmax, amax)
    margin = 1.000001
    samples, waypoint_indices, elapsed = [], [], 0.0
    for h, controls in zip(durations, segments):
        duration = h * margin
        # Whole-curve derivative hull bounds, not point-to-point displacement,
        # bound spacing even around reversals. Controller quintic interpolation
        # reconstructs these segments from the sampled position/velocity/accel.
        divisions = max(4, math.ceil(duration / 0.05),
                        math.ceil(max(abs(v) * h for _, d, _ in controls for v in d) / 0.001))
        if len(samples) + divisions + 1 > 2048:
            raise ValueError("Chunk exceeds 2048 dense trajectory samples")
        for n in range(0 if not samples else 1, divisions + 1):
            u = n / divisions
            samples.append((elapsed + u * duration,
                            [_bezier(b, u) for b, _, _ in controls],
                            [_bezier(d, u) / margin for _, d, _ in controls],
                            [_bezier(dd, u) / margin ** 2 for _, _, dd in controls]))
        waypoint_indices.append(len(samples) - 1)
        elapsed += duration
    effective = [h * margin for h in durations]
    return samples, {"actions_per_chunk": len(durations), "timing_mode": "adaptive",
                     "nominal_action_period_seconds": period,
                     "time_scale": elapsed / (period * len(durations)),
                     "effective_action_period_seconds": elapsed / len(durations),
                     "segment_durations_seconds": effective,
                     "segment_time_scales": [h / period for h in effective],
                     "waypoint_sample_indices": waypoint_indices}


def timed_chunk(current, waypoints, period, velocity_limits, acceleration_limits, *, timing_mode="uniform"):
    """Return dense (time, positions, velocities, accelerations) samples.

    Derivative control-point bounds enforce limits over the entire polynomial,
    not just at sample times. Position control points remain in each segment's
    endpoint interval. Uniform timing preserves relative waypoint timing.
    Optional adaptive timing changes segment durations and retains all targets.
    """
    positions = [joint_vector(list(current), "start")] + [joint_vector(q, "waypoint") for q in waypoints]
    h = number(period, "period")
    if not 0.01 <= h <= 1.0 or not 2 <= len(waypoints) <= 32:
        raise ValueError("Expected 2..32 waypoints and a 0.01..1 second period")
    vmax = joint_vector(list(velocity_limits), "velocity limits")
    amax = joint_vector(list(acceleration_limits), "acceleration limits")
    if min(vmax + amax) <= 0:
        raise ValueError("Positive velocity and acceleration limits are required")
    if timing_mode not in {"uniform", "adaptive"}:
        raise ValueError("Unknown chunk timing mode")
    if timing_mode == "adaptive":
        return _sample_adaptive(positions, h, vmax, amax)
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
    return samples, {"actions_per_chunk": len(waypoints), "timing_mode": "uniform", "nominal_action_period_seconds": h,
                     "time_scale": stretch, "effective_action_period_seconds": duration,
                     "waypoint_sample_indices": waypoint_indices}
