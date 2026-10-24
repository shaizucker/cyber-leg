"""Read knee settings without enable, zero, movement, Rest or flash commands."""
import argparse
import json
from motorevo_driver import MotorevoMotor
from knee_safety import read_configuration, validate_configuration


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--channel', default='PCAN_USBBUS1')
    args = parser.parse_args()
    motor = MotorevoMotor(channel=args.channel)
    report = dict(physical_zero_verified=False, clockwise_direction_verified=False,
                  power_cycle_persistence_verified=False, physical_travel_guaranteed=False)
    try:
        report['configuration'] = read_configuration(motor)
        validate_configuration(report['configuration'])
        report['configuration_matches_bridge'] = True
        result = 0
    except Exception as exc:
        report['configuration_matches_bridge'] = False
        report['error'] = str(exc)
        result = 2
    finally:
        # close() would send Rest; this audit intentionally sends only reads.
        motor.bus.shutdown()
    print(json.dumps(report, indent=2))
    return result


if __name__ == '__main__':
    raise SystemExit(main())
