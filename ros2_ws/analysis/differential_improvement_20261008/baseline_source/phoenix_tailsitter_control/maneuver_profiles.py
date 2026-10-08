"""Opt-in SITL tuning; standard keeps the previously tested controller path."""

CONTROLLER_PROFILES = {
    'standard': {
        'ground_only_takeoff_force_floor': False,
        'matched_moment_filter_enabled': False,
        'maneuver_acceleration_limit_scale': 1.0,
        'maneuver_moment_limit_scale': 1.0,
    },
    'knife-edge-transition': {
        'ground_only_takeoff_force_floor': True,
        'matched_moment_filter_enabled': True,
        'maneuver_acceleration_limit_scale': 4.0,
        'maneuver_moment_limit_scale': 4.0,
    },
}


def controller_arguments(name):
    """Explicit values keep experiment manifests independently reproducible."""
    values = CONTROLLER_PROFILES[name]
    return [f'{key}:={str(value).lower()}' for key, value in values.items()]


def mission_arguments(name):
    return ['knife_entry_straight:=true'] if name == 'knife-edge-transition' else []
