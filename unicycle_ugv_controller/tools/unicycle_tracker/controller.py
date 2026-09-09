"""Unicycle pose velocity, chassis box, and Custom1 flatness.

C++ runtime must stay in lockstep with this module.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import atan2, cos, hypot, isfinite, pi, sin
from typing import Optional, Tuple


def wrap_angle(value: float) -> float:
    return atan2(sin(value), cos(value))


def clamp(value: float, min_value: float, max_value: float) -> float:
    return max(min_value, min(max_value, value))


def box_saturate(
    linear_speed: float,
    angular_speed: float,
    *,
    max_linear_speed: float = 1.05,
    max_yaw_rate: float = 1.05,
) -> tuple[float, float]:
    return (
        clamp(linear_speed, -max_linear_speed, max_linear_speed),
        clamp(angular_speed, -max_yaw_rate, max_yaw_rate),
    )


def body_speed_from_world(yaw: float, vx: float, vy: float) -> float:
    return cos(yaw) * vx + sin(yaw) * vy


@dataclass
class PoseVelocityFilter:
    x1: float = 0.0
    x2: float = 0.0
    initialized: bool = False


def update_pose_velocity_filter(
    filt: PoseVelocityFilter,
    input_u: float,
    dt: float,
    *,
    wn: float = 31.41592653589793,
    zeta: float = 0.7071067811865476,
    dt_min: float = 1.0e-4,
    dt_max: float = 0.2,
) -> bool:
    if not (dt_min < dt <= dt_max) or wn <= 0.0 or zeta <= 0.0:
        return False
    if not filt.initialized:
        filt.x1 = input_u
        filt.x2 = 0.0
        filt.initialized = True
        return False
    a00, a01, a10, a11 = 0.0, 1.0, -wn * wn, -2.0 * zeta * wn
    b0, b1 = 0.0, wn * wn
    ad00, ad01, ad10, ad11 = _expm2x2(a00 * dt, a01 * dt, a10 * dt, a11 * dt)
    det = a00 * a11 - a01 * a10
    if abs(det) <= 1.0e-18:
        return False
    inv00, inv01, inv10, inv11 = a11 / det, -a01 / det, -a10 / det, a00 / det
    d00, d01, d10, d11 = ad00 - 1.0, ad01, ad10, ad11 - 1.0
    ia00 = inv00 * d00 + inv01 * d10
    ia01 = inv00 * d01 + inv01 * d11
    ia10 = inv10 * d00 + inv11 * d10
    ia11 = inv10 * d01 + inv11 * d11
    bd0 = ia00 * b0 + ia01 * b1
    bd1 = ia10 * b0 + ia11 * b1
    x1 = ad00 * filt.x1 + ad01 * filt.x2 + bd0 * input_u
    x2 = ad10 * filt.x1 + ad11 * filt.x2 + bd1 * input_u
    filt.x1 = x1
    filt.x2 = x2
    return True


def _expm2x2(a00: float, a01: float, a10: float, a11: float) -> tuple[float, float, float, float]:
    s00, s01, s10, s11 = a00, a01, a10, a11
    squares = 0
    norm = abs(a00) + abs(a01) + abs(a10) + abs(a11)
    while norm > 0.5:
        s00 *= 0.5
        s01 *= 0.5
        s10 *= 0.5
        s11 *= 0.5
        norm *= 0.5
        squares += 1
    e00, e01, e10, e11 = 1.0, 0.0, 0.0, 1.0
    t00, t01, t10, t11 = 1.0, 0.0, 0.0, 1.0
    factorial = 1.0
    for k in range(1, 13):
        n00 = t00 * s00 + t01 * s10
        n01 = t00 * s01 + t01 * s11
        n10 = t10 * s00 + t11 * s10
        n11 = t10 * s01 + t11 * s11
        t00, t01, t10, t11 = n00, n01, n10, n11
        factorial *= float(k)
        e00 += t00 / factorial
        e01 += t01 / factorial
        e10 += t10 / factorial
        e11 += t11 / factorial
    for _ in range(squares):
        n00 = e00 * e00 + e01 * e10
        n01 = e00 * e01 + e01 * e11
        n10 = e10 * e00 + e11 * e10
        n11 = e10 * e01 + e11 * e11
        e00, e01, e10, e11 = n00, n01, n10, n11
    return e00, e01, e10, e11


@dataclass(frozen=True)
class FlatnessOutput:
    linear_speed: float
    angular_speed: float
    accel: float
    valid: bool


def flatness_command(
    x: float,
    y: float,
    yaw: float,
    vx: float,
    vy: float,
    body_speed: float,
    ref_x: float,
    ref_y: float,
    ref_vx: float,
    ref_vy: float,
    ref_ax: float,
    ref_ay: float,
    dt: float,
    *,
    kp: float = 6.0,
    kv: float = 4.0,
    v_eps: float = 0.15,
    lateral_response_length: float = 0.8,
    lateral_damping: float = 1.0,
    max_linear_speed: float = 1.05,
    max_yaw_rate: float = 1.05,
    dt_min: float = 1.0e-4,
    dt_max: float = 0.2,
) -> FlatnessOutput:
    if not (dt_min < dt <= dt_max) or not (
        isfinite(lateral_response_length) and lateral_response_length > 0
        and isfinite(lateral_damping) and lateral_damping > 0
    ):
        return FlatnessOutput(0.0, 0.0, 0.0, False)
    ux = ref_ax + kv * (ref_vx - vx) + kp * (ref_x - x)
    uy = ref_ay + kv * (ref_vy - vy) + kp * (ref_y - y)
    c = cos(yaw)
    s = sin(yaw)
    accel = c * ux + s * uy
    bandwidth = abs(body_speed) / lateral_response_length
    ep = -s * (ref_x - x) + c * (ref_y - y)
    ev = -s * (ref_vx - vx) + c * (ref_vy - vy)
    lateral_accel = -s * ref_ax + c * ref_ay + 2 * lateral_damping * bandwidth * ev + bandwidth**2 * ep
    omega = body_speed * lateral_accel / (body_speed**2 + v_eps**2)
    speed, omega = box_saturate(
        body_speed + accel * dt, omega, max_linear_speed=max_linear_speed, max_yaw_rate=max_yaw_rate
    )
    return FlatnessOutput(speed, omega, accel, True)
