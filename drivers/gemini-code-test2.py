import time
import math
from motorevo_driver import MotorevoMotor

def test_motor_id1():
    print("Initializing MotorevoMotor for ID 1...")
    # Initialize using the exact same factory pattern as knee_bridge.py
    motor = MotorevoMotor(
        motor_id=1, 
        interface='pcan', 
        channel='PCAN_USBBUS1', 
        bitrate=1000000
    )

    try:
        # 1. Enter Rest Mode & Check Faults
        print("Entering rest mode...")
        t = motor.enter_rest_mode()
        print(f"Rest Status: {t}")
        if not t or t['fault'] != 0:
            raise RuntimeError(f"Motor has an active fault: {t.get('fault')}")

        # 2. Enter Motor Mode (This should light up the blue LED!)
        print("Entering motor mode (enabling FOC)...")
        t = motor.enter_motor_mode()
        print(f"Active Status: {t}")
        if not t or t['status'] != 1 or t['fault'] != 0:
            raise RuntimeError("Failed to enter motor mode safely.")

        print("Motor is ACTIVE! Holding position for 2 seconds...")
        start_pos = t['position_rad']
        
        # Hold position or gentle movement within 0 to 1.83 rad (105 deg)
        for i in range(100):
            # Small sinusoidal movement around start position (clamped to 0 - 1.83 rad)
            target = start_pos + 0.2 * math.sin(i * 0.1)
            target = max(0.0, min(target, 1.83))  # Strict safety bounds for 105°
            
            feedback = motor.send_servo_command(
                setpoint=target, 
                vel_limit_rad_s=2.0, 
                kp_pos=40.0, 
                kd_pos=10.0, 
                kp_vel=50.0, 
                kd_vel=0.0, 
                ki_vel=0.001
            )
            print(f"Target: {math.degrees(target):.1f}° | Actual: {math.degrees(feedback['position_rad']):.1f}° | Fault: {feedback['fault']}")
            time.sleep(0.02)

    except Exception as e:
        print(f"Error during test: {e}")
    finally:
        print("Returning motor to rest mode / closing...")
        try:
            motor.enter_rest_mode()
            motor.close()
        except:
            pass

if __name__ == '__main__':
    test_motor_id1()