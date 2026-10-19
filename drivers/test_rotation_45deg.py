"""
Smooth 45-degree rotation test for Motorevo Knee Actuator via PEAK-CAN
Ramps smoothly from 0° -> +45° -> hold -> return to 0° -> safe disarm.
"""

import time
import math
import can

def float_to_uint(val, val_min, val_max, bits):
    clamped = max(min(val, val_max), val_min)
    span = val_max - val_min
    return int((clamped - val_min) * ((1 << bits) - 1) / span)

def uint_to_float(raw, val_min, val_max, bits):
    span = val_max - val_min
    return val_min + (raw * span / ((1 << bits) - 1))

def unpack_feedback(data):
    if len(data) != 8:
        return None
    pos_raw = (data[1] << 8) | data[2]
    vel_raw = (data[3] << 4) | (data[4] >> 4)
    tau_raw = ((data[4] & 0x0F) << 8) | data[5]
    return {
        "status": data[0],
        "pos_rad": uint_to_float(pos_raw, -12.5, 12.5, 16),
        "vel_rad_s": uint_to_float(vel_raw, -10.0, 10.0, 12),
        "tau_nm": uint_to_float(tau_raw, -50.0, 50.0, 12),
        "fault": data[6],
        "temp": data[7]
    }

def make_servo_packet(target_pos_rad, vel_limit=3.0, kp_pos=45.0, kd_pos=5.0, kp_vel=50.0, kd_vel=0.0, ki_vel=0.005):
    p = float_to_uint(target_pos_rad, -12.5, 12.5, 16)
    v = float_to_uint(vel_limit, -10.0, 10.0, 8)
    kp_p = float_to_uint(kp_pos, 0.0, 250.0, 8)
    kd_p = float_to_uint(kd_pos, 0.0, 50.0, 8)
    kp_v = float_to_uint(kp_vel, 0.0, 250.0, 8)
    kd_v = float_to_uint(kd_vel, 0.0, 50.0, 8)
    ki_v = float_to_uint(ki_vel, 0.0, 0.05, 8)
    return [(p >> 8) & 0xFF, p & 0xFF, v, kp_p, kd_p, kp_v, kd_v, ki_v]

def main():
    print("=" * 65)
    print(" Motorevo Actuator: 45° Smooth Rotation Test")
    print("=" * 65)

    bus = can.Bus(interface='pcan', channel='PCAN_USBBUS1', bitrate=1000000)
    
    # Flush existing frames
    while bus.recv(timeout=0.01): pass

    # 1. Clear faults with Enter Rest State
    print("[1/5] Checking baseline status...")
    bus.send(can.Message(arbitration_id=1, data=[0xFF]*7 + [0xFD], is_extended_id=False))
    time.sleep(0.02)
    resp = bus.recv(timeout=0.05)
    if resp:
        telem = unpack_feedback(resp.data)
        print(f"      Initial Pos: {math.degrees(telem['pos_rad']):.2f}°, Fault: {telem['fault']}")
    else:
        print("      [ERROR] No CAN response from motor.")
        bus.shutdown()
        return

    # 2. Zero the position
    print("[2/5] Setting current angle as 0.0°...")
    bus.send(can.Message(arbitration_id=1, data=[0xFF]*7 + [0xFE], is_extended_id=False))
    time.sleep(0.05)

    # 3. Arm the motor into Motor State (releases brake)
    print("[3/5] Arming motor (Enter Motor Mode)...")
    bus.send(can.Message(arbitration_id=1, data=[0xFF]*7 + [0xFC], is_extended_id=False))
    time.sleep(0.05)
    resp = bus.recv(timeout=0.05)
    if resp:
        telem = unpack_feedback(resp.data)
        print(f"      Status: {telem['status']} (1=Active Motor State), Fault: {telem['fault']}")
        if telem['status'] != 1:
            print("      [ERROR] Motor failed to arm into Motor State!")
            bus.shutdown()
            return

    # 4. Trajectory Generation: 0° -> +45° (2.0s) -> Hold (1.0s) -> 0° (2.0s)
    target_max_rad = math.radians(45.0)  # ~0.785 rad
    dt = 0.02  # 50 Hz control loop
    
    # Trajectory phases
    traj = []
    
    # Ramp up: 0 -> 45 deg over 2.0s (100 steps)
    steps_ramp = 100
    for i in range(steps_ramp):
        s = 0.5 * (1 - math.cos(math.pi * i / steps_ramp))  # smooth S-curve
        traj.append(s * target_max_rad)
        
    # Hold at 45 deg for 1.0s (50 steps)
    for _ in range(50):
        traj.append(target_max_rad)
        
    # Ramp down: 45 -> 0 deg over 2.0s (100 steps)
    for i in range(steps_ramp):
        s = 0.5 * (1 + math.cos(math.pi * i / steps_ramp))
        traj.append(s * target_max_rad)

    print(f"[4/5] Executing smooth trajectory ({len(traj)*dt:.1f}s total at 50Hz)...")
    print("      Target(°) | Actual(°) | Vel(rad/s) | Torque(Nm)")
    print("      " + "-" * 50)

    try:
        for idx, target_rad in enumerate(traj):
            pkt = make_servo_packet(target_rad, vel_limit=3.0, kp_pos=50.0, kd_pos=6.0, kp_vel=60.0, kd_vel=0.0, ki_vel=0.005)
            bus.send(can.Message(arbitration_id=1, data=pkt, is_extended_id=False))
            
            resp = bus.recv(timeout=dt)
            if resp:
                t = unpack_feedback(resp.data)
                if idx % 15 == 0 or idx == len(traj) - 1:
                    print(f"      {math.degrees(target_rad):+7.1f}° | {math.degrees(t['pos_rad']):+7.1f}° | {t['vel_rad_s']:+9.2f} | {t['tau_nm']:+8.2f} Nm")
            
            time.sleep(dt)

    except KeyboardInterrupt:
        print("\n[!] Aborted by user.")

    finally:
        # 5. Safe disarm
        print("[5/5] Disarming motor (Brake engages, Rest Mode)...")
        bus.send(can.Message(arbitration_id=1, data=[0xFF]*7 + [0xFD], is_extended_id=False))
        time.sleep(0.05)
        resp = bus.recv(timeout=0.05)
        if resp:
            t = unpack_feedback(resp.data)
            print(f"      Final Position: {math.degrees(t['pos_rad']):.2f}°, Status: {t['status']}")
        bus.shutdown()
        print("Done!")

if __name__ == "__main__":
    main()
