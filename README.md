# 载具动力学：插值、局部摩擦与确定性同步

斜坡与网络延迟下载具姿态对不上：渲染不插值、摩擦用世界坐标、悬挂射线漏动态层、同步不发四元数、倒车转向反、缩放后步数取整、唤醒清零。本仓库固定这些契约。

- 领域：游戏、图形与多媒体
- 子方向：图形渲染
- 语言/框架：Python 3, pytest
- 初始状态：15 个参数化用例全部失败

## 目录结构

```
README.md
conftest.py
.gitignore
gameplay/__init__.py
gameplay/vehicle_dynamics.py
tests/test_vehicle_dynamics.py
```

## 需要修复的 10 个问题

1. 渲染按固定步长物理结果插值，斜坡上不抖
2. 轮胎摩擦在车身局部坐标计算，旋转后侧向力方向正确
3. 悬挂射线包含动态层，压到箱子不穿透
4. 网络同步同时发送位置与四元数，回放车头不乱转
5. 倒车时转向输入取反，方向盘与车头一致
6. 多相机共享同一份预测位姿，分屏不漂移
7. 时间缩放后按累积器取物理步数，慢动作加速度不变
8. 睡眠刚体被碰撞唤醒时保留速度，不清零
9. 固定步长确定性积分，不同客户端同一坡道距离一致
10. 输入缓冲按帧去重，丢包时不重复应用同一帧油门

## 15 条验收契约

    01. render interpolation — `interpolate_pose`
    02. local friction rotation — `local_friction`
    03. dynamic layer ray — `raycast_layers`
    04. quaternion pose sync — `sync_quaternion`
    05. reverse steering — `steering_input`
    06. shared camera prediction — `camera_prediction`
    07. time scaled steps — `physics_steps`
    08. wake preserves velocity — `wake_velocity`
    09. deterministic integration — `integrate_velocity`
    10. input frame dedup — `dedupe_input`
    11. slope gravity — `slope_accel`
    12. suspension spring — `suspension_force`
    13. pose snapshot — `snapshot_pose`
    14. accumulator steps — `accumulator_steps`
    15. steering clamp — `clamp_steer`

## 当前状态

`gameplay/vehicle_dynamics.py` 只保留函数签名与文档字符串，所有函数体统一
`raise NotImplementedError`。`tests/test_vehicle_dynamics.py` 使用一个
`@pytest.mark.parametrize` 装载 15 条契约，初始全部失败且没有 collection error。

```bash
python -m pytest -q
```

预期输出包含 `15 failed`。实现 `gameplay/vehicle_dynamics.py` 后，15 条契约应全部通过。
