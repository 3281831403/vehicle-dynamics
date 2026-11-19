"""载具动力学：插值、局部摩擦与确定性同步。"""

from __future__ import annotations

import math

DEFAULT_FIXED_DT = 1.0 / 60.0


def _normalize_quaternion(quat):
    """把 ``(x, y, z, w)`` 四元数归一化；退化输入返回单位四元数。"""
    if len(quat) != 4:
        raise ValueError("quaternion must have 4 components")
    x, y, z, w = (float(c) for c in quat)
    length = math.sqrt(x * x + y * y + z * z + w * w)
    if length == 0.0 or not math.isfinite(length):
        return (0.0, 0.0, 0.0, 1.0)
    return (x / length, y / length, z / length, w / length)


def _lerp(start, end, alpha):
    return start + (end - start) * alpha


def interpolate_pose(prev_pose, curr_pose, alpha):
    """位置与朝向的渲染插值。

    位姿为 ``(x, y, z, yaw_deg)``；朝向上按最短弧插值，避免在 ±180° 附近
    绕远路造成渲染抖动。``alpha`` 会被钳制到 ``[0, 1]``。
    """
    if len(prev_pose) != 4 or len(curr_pose) != 4:
        raise ValueError("pose must be (x, y, z, yaw_deg)")
    alpha = min(1.0, max(0.0, float(alpha)))
    x = _lerp(prev_pose[0], curr_pose[0], alpha)
    y = _lerp(prev_pose[1], curr_pose[1], alpha)
    z = _lerp(prev_pose[2], curr_pose[2], alpha)
    delta = (curr_pose[3] - prev_pose[3] + 180.0) % 360.0 - 180.0
    yaw = prev_pose[3] + delta * alpha
    return (x, y, z, yaw)


def local_friction(force_world, heading_deg):
    """把世界摩擦变换到车身局部坐标。

    车身朝向为 ``heading_deg``（绕 Z 轴、逆时针为正），因此世界→车身的
    变换是旋转 ``-heading``。
    """
    fx, fy = float(force_world[0]), float(force_world[1])
    angle = -math.radians(heading_deg)
    cos_a, sin_a = math.cos(angle), math.sin(angle)
    return (fx * cos_a - fy * sin_a, fx * sin_a + fy * cos_a)


def raycast_layers(mask, dynamic_bit):
    """悬挂射线是否命中动态层（即射线遮罩是否包含动态层位）。

    返回 ``True`` 表示射线应把动态刚体也纳入检测，而不是只打静态层。
    """
    return bool(int(mask) & int(dynamic_bit))


def sync_quaternion(quat):
    """归一化并同步四元数姿态。"""
    return _normalize_quaternion(quat)


def steering_input(steer, reverse):
    """倒车时转向输入取反，保证方向盘与车头转向一致。"""
    steer = float(steer)
    return -steer if reverse else steer


def camera_prediction(pose, camera_count):
    """返回多个相机共享的预测位姿。

    同一帧内所有视口拿到同一份预测结果，分屏时载具不会出现在两处。
    """
    count = int(camera_count)
    if count < 0:
        raise ValueError("camera_count must be non-negative")
    return tuple(pose for _ in range(count))


def physics_steps(dt, fixed_dt, time_scale):
    """缩放时间下的整数物理步数与余量。

    先把缩放后的时长 ``dt * time_scale`` 送进固定步长累积器，得到完整的
    步数与剩余的亚步长时间；慢动作下每帧走更少的步，但单步加速度不变。
    """
    fixed_dt = float(fixed_dt)
    if fixed_dt <= 0.0:
        raise ValueError("fixed_dt must be positive")
    scaled_dt = float(dt) * float(time_scale)
    if scaled_dt < 0.0:
        raise ValueError("dt * time_scale must be non-negative")
    steps = int(math.floor(scaled_dt / fixed_dt))
    remainder = scaled_dt - steps * fixed_dt
    return (steps, remainder)


def wake_velocity(velocity, impact):
    """唤醒时保留并叠加碰撞速度（而不是把速度清零）。"""
    return float(velocity) + float(impact)


def integrate_velocity(velocity, accel, dt):
    """确定性速度积分。

    使用 ``math.fsum`` 做一次无中间舍入的求和，固定步长下各客户端得到
    逐位一致的结果。
    """
    return math.fsum((float(velocity), float(accel) * float(dt)))


def dedupe_input(frame, last_frame):
    """输入帧是否为新帧；与上一帧相同则丢弃，避免丢包重放重复施加。"""
    return last_frame is None or int(frame) != int(last_frame)


def slope_accel(gravity, slope_deg):
    """斜坡重力分量（沿坡面向下为正，因此上坡为负）。"""
    return -float(gravity) * math.sin(math.radians(float(slope_deg)))


def suspension_force(compression, stiffness):
    """悬挂弹簧力；悬挂离地（压缩量为负）时不产生拉力。"""
    return max(0.0, float(compression)) * float(stiffness)


def snapshot_pose(position, quat):
    """网络姿态快照：位置与归一化四元数一并发送。"""
    return (tuple(float(c) for c in position), _normalize_quaternion(quat))


def accumulator_steps(accumulator, dt, fixed_dt):
    """固定步长累积器返回步数与余量。

    ``accumulator`` 中的残留时间会被保留到下一帧，避免时间缩放或帧率
    波动时步数被取整丢掉。
    """
    fixed_dt = float(fixed_dt)
    if fixed_dt <= 0.0:
        raise ValueError("fixed_dt must be positive")
    total = float(accumulator) + float(dt)
    if total < 0.0:
        raise ValueError("accumulator + dt must be non-negative")
    steps = int(math.floor(total / fixed_dt))
    remainder = total - steps * fixed_dt
    return (steps, remainder)


def clamp_steer(steer, max_steer):
    """转向输入钳制到对称区间 ``[-max_steer, max_steer]``。"""
    limit = abs(float(max_steer))
    return min(limit, max(-limit, float(steer)))
