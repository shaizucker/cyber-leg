"""Checks of reported settings; these do not prove physical travel containment."""
import math

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


def validate_configuration(config, theta_min=0.0, theta_max=1.83):
    for name in ('machine_offset_rad', 'theta_min_rad', 'theta_max_rad'):
        if not math.isfinite(config[name]):
            raise RuntimeError(f'Knee {name} is not finite')
    if config['control_mode'] != 2:
        raise RuntimeError('Knee parameters must select P-T-M mode (2)')
    for name, expected in (('theta_min_rad', theta_min), ('theta_max_rad', theta_max)):
        if abs(config[name] - expected) > 1e-6:
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
