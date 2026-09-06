"""Unicycle T24 Reset, T25 pose velocity, T26 box, and Custom1 flatness.

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


_RESET_SAMPLES = 48
_OMEGA_DENOM_EPS = 1.0e-8
_MIN_RESET_DURATION = 0.05


@dataclass
class BezierPlan:
    p0x: float = 0.0
    p0y: float = 0.0
    p1x: float = 0.0
    p1y: float = 0.0
    p2x: float = 0.0
    p2y: float = 0.0
    p3x: float = 0.0
    p3y: float = 0.0
    T: float = 0.0
    reverse: bool = False
    valid: bool = False
    already_arrived: bool = False
    fence_failed: bool = False


@dataclass(frozen=True)
class ResetSample:
    x: float
    y: float
    yaw: float
    linear_speed: float
    angular_speed: float
    valid: bool


def _bezier_point(plan: BezierPlan, s: float) -> tuple[float, float]:
    u = 1.0 - s
    uu, uuu, ss, sss = u * u, u * u * u, s * s, s * s * s
    x = uuu * plan.p0x + 3.0 * uu * s * plan.p1x + 3.0 * u * ss * plan.p2x + sss * plan.p3x
    y = uuu * plan.p0y + 3.0 * uu * s * plan.p1y + 3.0 * u * ss * plan.p2y + sss * plan.p3y
    return x, y


def _bezier_deriv(plan: BezierPlan, s: float) -> tuple[float, float]:
    u = 1.0 - s
    dx = (
        3.0 * u * u * (plan.p1x - plan.p0x)
        + 6.0 * u * s * (plan.p2x - plan.p1x)
        + 3.0 * s * s * (plan.p3x - plan.p2x)
    )
    dy = (
        3.0 * u * u * (plan.p1y - plan.p0y)
        + 6.0 * u * s * (plan.p2y - plan.p1y)
        + 3.0 * s * s * (plan.p3y - plan.p2y)
    )
    return dx, dy


def _bezier_second(plan: BezierPlan, s: float) -> tuple[float, float]:
    u = 1.0 - s
    ddx = 6.0 * u * (plan.p2x - 2.0 * plan.p1x + plan.p0x) + 6.0 * s * (
        plan.p3x - 2.0 * plan.p2x + plan.p1x
    )
    ddy = 6.0 * u * (plan.p2y - 2.0 * plan.p1y + plan.p0y) + 6.0 * s * (
        plan.p3y - 2.0 * plan.p2y + plan.p1y
    )
    return ddx, ddy


def kinematics_at(
    plan: BezierPlan, s: float, duration: float
) -> Optional[Tuple[float, float, float, float, float]]:
    if duration <= 0.0:
        return None
    x, y = _bezier_point(plan, s)
    dp_x, dp_y = _bezier_deriv(plan, s)
    ddp_x, ddp_y = _bezier_second(plan, s)
    vx, vy = dp_x / duration, dp_y / duration
    ax, ay = ddp_x / (duration * duration), ddp_y / (duration * duration)
    speed_sq = vx * vx + vy * vy
    if speed_sq < _OMEGA_DENOM_EPS:
        return None
    omega = (vx * ay - vy * ax) / speed_sq
    tangent_yaw = atan2(vy, vx)
    yaw = wrap_angle(tangent_yaw + pi) if plan.reverse else wrap_angle(tangent_yaw)
    speed = hypot(vx, vy)
    v = copysign_speed(speed, vx * cos(yaw) + vy * sin(yaw))
    if not (v == v and omega == omega and yaw == yaw):
        return None
    return v, omega, x, y, yaw


def copysign_speed(speed: float, along: float) -> float:
    if along < 0.0:
        return -speed
    return speed


def plan_feasible(
    plan: BezierPlan,
    duration: float,
    *,
    max_linear_speed: float,
    max_yaw_rate: float,
) -> bool:
    if duration <= 0.0:
        return False
    for i in range(_RESET_SAMPLES + 1):
        s = i / float(_RESET_SAMPLES)
        kin = kinematics_at(plan, s, duration)
        if kin is None:
            if i == 0 or i == _RESET_SAMPLES:
                continue
            return False
        v, omega, _, _, _ = kin
        if abs(v) > max_linear_speed + 1.0e-9 or abs(omega) > max_yaw_rate + 1.0e-9:
            return False
    return True


def path_inside_fence(
    plan: BezierPlan,
    *,
    x_min: float = -20.0,
    x_max: float = 20.0,
    y_min: float = -20.0,
    y_max: float = 20.0,
) -> bool:
    for i in range(_RESET_SAMPLES + 1):
        s = i / float(_RESET_SAMPLES)
        x, y = _bezier_point(plan, s)
        if not (x_min <= x <= x_max and y_min <= y <= y_max):
            return False
    return True


def _make_candidate(
    x: float, y: float, yaw: float, goal_x: float, goal_y: float, goal_yaw: float, length: float, reverse: bool
) -> BezierPlan:
    start_yaw = wrap_angle(yaw + pi) if reverse else yaw
    end_yaw = wrap_angle(goal_yaw + pi) if reverse else goal_yaw
    return BezierPlan(
        p0x=x,
        p0y=y,
        p1x=x + length * cos(start_yaw),
        p1y=y + length * sin(start_yaw),
        p2x=goal_x - length * cos(end_yaw),
        p2y=goal_y - length * sin(end_yaw),
        p3x=goal_x,
        p3y=goal_y,
        reverse=reverse,
    )


def _search_duration(
    plan: BezierPlan,
    *,
    max_linear_speed: float,
    max_yaw_rate: float,
    reset_timeout: float,
) -> bool:
    dist = hypot(plan.p3x - plan.p0x, plan.p3y - plan.p0y)
    t_hi = max(_MIN_RESET_DURATION, dist / max(max_linear_speed, 1.0e-6))
    t_cap = reset_timeout if reset_timeout > 0.0 else 45.0
    if not plan_feasible(plan, t_hi, max_linear_speed=max_linear_speed, max_yaw_rate=max_yaw_rate):
        found = False
        while t_hi < t_cap:
            t_hi = min(t_hi * 2.0, t_cap)
            if plan_feasible(plan, t_hi, max_linear_speed=max_linear_speed, max_yaw_rate=max_yaw_rate):
                found = True
                break
            if t_hi >= t_cap:
                break
        if not found:
            return False
    t_lo = _MIN_RESET_DURATION
    for _ in range(24):
        mid = 0.5 * (t_lo + t_hi)
        if plan_feasible(plan, mid, max_linear_speed=max_linear_speed, max_yaw_rate=max_yaw_rate):
            t_hi = mid
        else:
            t_lo = mid
    plan.T = t_hi
    plan.valid = plan_feasible(plan, plan.T, max_linear_speed=max_linear_speed, max_yaw_rate=max_yaw_rate)
    return plan.valid


def plan_reset(
    x: float,
    y: float,
    yaw: float,
    goal_x: float,
    goal_y: float,
    goal_yaw: float,
    *,
    arrive_position: float = 0.05,
    max_linear_speed: float = 1.05,
    max_yaw_rate: float = 1.05,
    reset_timeout: float = 45.0,
    x_min: float = -20.0,
    x_max: float = 20.0,
    y_min: float = -20.0,
    y_max: float = 20.0,
) -> BezierPlan:
    plan = BezierPlan()
    if not (x_min <= x <= x_max and y_min <= y <= y_max):
        plan.fence_failed = True
        return plan
    dist = hypot(goal_x - x, goal_y - y)
    if dist <= arrive_position:
        plan.already_arrived = True
        plan.valid = True
        plan.p0x, plan.p0y = x, y
        plan.p3x, plan.p3y = goal_x, goal_y
        return plan
    length = dist / 3.0
    best = BezierPlan(T=reset_timeout + 1.0 if reset_timeout > 0.0 else 1.0e9)
    any_feasible = False
    for _ in range(5):
        for reverse in (False, True):
            candidate = _make_candidate(x, y, yaw, goal_x, goal_y, goal_yaw, length, reverse)
            if not path_inside_fence(
                candidate, x_min=x_min, x_max=x_max, y_min=y_min, y_max=y_max
            ):
                continue
            if not _search_duration(
                candidate,
                max_linear_speed=max_linear_speed,
                max_yaw_rate=max_yaw_rate,
                reset_timeout=reset_timeout,
            ):
                continue
            any_feasible = True
            if candidate.T < best.T:
                best = candidate
        if any_feasible:
            break
        length *= 0.5
    if not any_feasible:
        plan.fence_failed = True
        return plan
    return best


def sample_reset(plan: BezierPlan, t_along: float) -> ResetSample:
    if not plan.valid or plan.already_arrived or plan.T <= 0.0:
        return ResetSample(0.0, 0.0, 0.0, 0.0, 0.0, False)
    s = clamp(max(0.0, t_along) / plan.T, 0.0, 1.0)
    kin = kinematics_at(plan, s, plan.T)
    if kin is None:
        x, y = _bezier_point(plan, s)
        return ResetSample(x, y, 0.0, 0.0, 0.0, True)
    v, omega, x, y, yaw = kin
    return ResetSample(x, y, yaw, v, omega, True)


def max_box_on_plan(
    plan: BezierPlan, *, max_linear_speed: float = 1.05, max_yaw_rate: float = 1.05
) -> tuple[float, float, bool]:
    max_v = 0.0
    max_w = 0.0
    ok = True
    if not plan.valid or plan.already_arrived:
        return 0.0, 0.0, True
    for i in range(_RESET_SAMPLES + 1):
        s = i / float(_RESET_SAMPLES)
        kin = kinematics_at(plan, s, plan.T)
        if kin is None:
            continue
        v, omega, _, _, _ = kin
        max_v = max(max_v, abs(v))
        max_w = max(max_w, abs(omega))
        if abs(v) > max_linear_speed + 1.0e-6 or abs(omega) > max_yaw_rate + 1.0e-6:
            ok = False
    return max_v, max_w, ok
