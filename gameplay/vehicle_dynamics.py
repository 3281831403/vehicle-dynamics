"""载具动力学：插值、局部摩擦与确定性同步：待实现的核心模块。"""

from __future__ import annotations


def interpolate_pose(prev_pose, curr_pose, alpha):
    """位置与朝向的渲染插值。"""
    raise NotImplementedError

def local_friction(force_world, heading_deg):
    """把世界摩擦变换到车身局部坐标。"""
    raise NotImplementedError

def raycast_layers(mask, dynamic_bit):
    """悬挂射线是否命中动态层。"""
    raise NotImplementedError

def sync_quaternion(quat):
    """归一化并同步四元数姿态。"""
    raise NotImplementedError

def steering_input(steer, reverse):
    """倒车时转向输入取反。"""
    raise NotImplementedError

def camera_prediction(pose, camera_count):
    """返回多个相机共享的预测位姿。"""
    raise NotImplementedError

def physics_steps(dt, fixed_dt, time_scale):
    """缩放时间下的整数物理步数与余量。"""
    raise NotImplementedError

def wake_velocity(velocity, impact):
    """唤醒时保留并叠加碰撞速度。"""
    raise NotImplementedError

def integrate_velocity(velocity, accel, dt):
    """确定性速度积分。"""
    raise NotImplementedError

def dedupe_input(frame, last_frame):
    """输入帧是否为新帧。"""
    raise NotImplementedError

def slope_accel(gravity, slope_deg):
    """斜坡重力分量。"""
    raise NotImplementedError

def suspension_force(compression, stiffness):
    """悬挂弹簧力。"""
    raise NotImplementedError

def snapshot_pose(position, quat):
    """网络姿态快照。"""
    raise NotImplementedError

def accumulator_steps(accumulator, dt, fixed_dt):
    """固定步长累积器返回步数与余量。"""
    raise NotImplementedError

def clamp_steer(steer, max_steer):
    """转向输入钳制。"""
    raise NotImplementedError
