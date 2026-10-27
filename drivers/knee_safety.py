"""Checks of reported settings; these do not prove physical travel containment."""
import math

# Motor protection may extend one degree past the bridge's operating envelope,
# or use the explicitly supported -12..12 rad servo setting.
# The actual parameter readback, rather than the command envelope, scales CAN.
POSITION_GUARD_RAD = math.radians(1.0)
WIDE_POSITION_RANGE_RAD = (-12.0, 12.0)

PARAMETERS = {
    'control_mode': (11, 'int'),
    'machine_offset_rad': (23, 'float'),
    'theta_min_rad': (24, 'float'),
    'theta_max_rad': (25, 'float'),
    'can_timeout_ms': (37, 'int'),
    'rotation_direction': (47, 'int'),
    'protection_mask': (56, 'uint'),
}


def read_configuration(motor):
    return {name: motor.read_parameter(index, kind) for name, (index, kind) in PARAMETERS.items()}


def verify_command_scaling(motor):
    """Online gains/torque are meaningful only with matching CAN conversion ranges."""
    expected = {26:motor.vel_min,27:motor.vel_max,28:motor.kp_min,29:motor.kp_max,
                30:motor.kd_min,31:motor.kd_max,34:motor.torque_min,35:motor.torque_max}
    for index,value in expected.items():
        actual = motor.read_parameter(index)
        if not math.isfinite(actual) or not math.isclose(actual,value,rel_tol=1e-6,abs_tol=1e-6):
            raise RuntimeError(f'Motor CAN scaling parameter {index} differs from bridge; do not arm')


def validate_configuration(config, theta_min=0.0, theta_max=1.83, guard_band=0.0, allow_wide_range=False):
    if not math.isfinite(guard_band) or guard_band < 0:
        raise ValueError('Position guard band must be finite and nonnegative')
    for name in ('machine_offset_rad', 'theta_min_rad', 'theta_max_rad'):
        if not math.isfinite(config[name]):
            raise RuntimeError(f'Knee {name} is not finite')
    if config['control_mode'] != 2:
        raise RuntimeError('Knee parameters must select P-T-M mode (2)')
    wide_range = allow_wide_range and all(
        math.isclose(config[name], expected, rel_tol=0, abs_tol=1e-6)
        for name, expected in zip(('theta_min_rad', 'theta_max_rad'), WIDE_POSITION_RANGE_RAD))
    for name, expected in (('theta_min_rad', theta_min), ('theta_max_rad', theta_max)):
        low, high = ((expected-guard_band, expected) if name == 'theta_min_rad'
                     else (expected, expected+guard_band))
        if not wide_range and not low-1e-6 <= config[name] <= high+1e-6:
            raise RuntimeError(f'Knee {name}={config[name]:.8f} does not match bridge scale {expected}; do not arm')
    if config['rotation_direction'] not in (-1, 1):
        raise RuntimeError('Knee Rotate Dir is invalid')
    if not config['protection_mask'] & 1:
        raise RuntimeError('Knee position-limit protection is DISABLED (Protect Switchs bit 0)')
    if not config['protection_mask'] & 2:
        raise RuntimeError('Knee overspeed protection is DISABLED (Protect Switchs bit 1)')
    if not config['protection_mask'] & 0x80 or config['can_timeout_ms'] <= 0:
        raise RuntimeError('Knee CAN timeout protection is disabled or invalid')
    return config
