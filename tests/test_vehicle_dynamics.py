import pytest

from gameplay import vehicle_dynamics as mod

CASES = [
    pytest.param('interpolate_pose', ((0.0, 0.0, 0.0, 0.0), (10.0, 0.0, 0.0, 90.0), 0.25), (2.5, 0.0, 0.0, 22.5), True, id='01_render_interpolation'),
    pytest.param('local_friction', ((1.0, 0.0), 90.0), (6.123233995736766e-17, -1.0), True, id='02_local_friction_rotation'),
    pytest.param('raycast_layers', (6, 2), True, False, id='03_dynamic_layer_ray'),
    pytest.param('sync_quaternion', ((0.0, 0.0, 0.0, 2.0),), (0.0, 0.0, 0.0, 1.0), True, id='04_quaternion_pose_sync'),
    pytest.param('steering_input', (0.5, True), -0.5, True, id='05_reverse_steering'),
    pytest.param('camera_prediction', ((1.0, 2.0, 3.0, 0.0), 2), ((1.0, 2.0, 3.0, 0.0), (1.0, 2.0, 3.0, 0.0)), True, id='06_shared_camera_prediction'),
    pytest.param('physics_steps', (0.05, 0.02, 0.5), (1, 0.005000000000000001), True, id='07_time_scaled_steps'),
    pytest.param('wake_velocity', (2.0, 1.0), 3.0, True, id='08_wake_preserves_velocity'),
    pytest.param('integrate_velocity', (1.0, 2.0, 0.5), 2.0, True, id='09_deterministic_integration'),
    pytest.param('dedupe_input', (7, 7), False, False, id='10_input_frame_dedup'),
    pytest.param('slope_accel', (9.81, 30.0), -4.904999999999999, True, id='11_slope_gravity'),
    pytest.param('suspension_force', (0.1, 50.0), 5.0, True, id='12_suspension_spring'),
    pytest.param('snapshot_pose', ((1.0, 2.0, 3.0), (0.0, 0.0, 0.0, 2.0)), ((1.0, 2.0, 3.0), (0.0, 0.0, 0.0, 1.0)), True, id='13_pose_snapshot'),
    pytest.param('accumulator_steps', (0.01, 0.025, 0.02), (1, 0.015000000000000003), True, id='14_accumulator_steps'),
    pytest.param('clamp_steer', (1.2, 0.8), 0.8, True, id='15_steering_clamp'),
]


def assert_close(actual, expected):
    if isinstance(expected, tuple):
        assert isinstance(actual, tuple) and len(actual) == len(expected)
        for a, e in zip(actual, expected):
            assert_close(a, e)
    elif isinstance(expected, list):
        assert isinstance(actual, list) and len(actual) == len(expected)
        for a, e in zip(actual, expected):
            assert_close(a, e)
    elif isinstance(expected, dict):
        assert actual.keys() == expected.keys()
        for key in expected:
            assert_close(actual[key], expected[key])
    else:
        assert actual == pytest.approx(expected)


@pytest.mark.parametrize("func_name,args,expected,approx", CASES)
def test_vehicle_dynamics_contract(func_name, args, expected, approx):
    func = getattr(mod, func_name)
    result = func(*args)
    if approx:
        assert_close(result, expected)
    else:
        assert result == expected
