import can
import time

def uint_to_float(val, val_min, val_max, bits):
    span = val_max - val_min
    return val_min + (val * span / ((1 << bits) - 1))

def unpack_feedback(data):
    status = data[0]
    pos_raw = (data[1] << 8) | data[2]
    vel_raw = (data[3] << 4) | (data[4] >> 4)
    tau_raw = ((data[4] & 0x0F) << 8) | data[5]
    fault = data[6]
    temp = data[7]

    pos = uint_to_float(pos_raw, -12.5, 12.5, 16)
    vel = uint_to_float(vel_raw, -10.0, 10.0, 12)
    torque = uint_to_float(tau_raw, -50.0, 50.0, 12)
    return {
        "status": status,
        "position_rad": pos,
        "velocity_rad_s": vel,
        "torque_nm": torque,
        "fault": fault,
        "temp_c": temp
    }

def main():
    print("Opening PCAN_USBBUS1 at 1 Mbps...")
    bus = can.Bus(interface='pcan', channel='PCAN_USBBUS1', bitrate=1000000)

    # Let's ping motor ID 1 with 'Enter Rest State' command (safe command, does not move motor)
    # [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD]
    rest_data = [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD]
    
    print("Sending ping (Enter Rest State) to Motor ID 1...")
    msg = can.Message(arbitration_id=1, data=rest_data, is_extended_id=False)
    bus.send(msg)

    # Wait for response
    start = time.time()
    response_received = False
    while time.time() - start < 1.0:
        resp = bus.recv(timeout=0.1)
        if resp:
            print(f"Received frame: ID=0x{resp.arbitration_id:X} Data={resp.data.hex(' ')}")
            if len(resp.data) == 8:
                telemetry = unpack_feedback(resp.data)
                print(f"--> Status: {telemetry['status']} (0=Rest, 1=Servo, 2=P-T-M)")
                print(f"--> Position: {telemetry['position_rad']:.3f} rad ({telemetry['position_rad']*57.2958:.1f} deg)")
                print(f"--> Velocity: {telemetry['velocity_rad_s']:.3f} rad/s")
                print(f"--> Torque: {telemetry['torque_nm']:.3f} Nm")
                print(f"--> Fault Code: {telemetry['fault']} (0=No fault)")
                print(f"--> Temperature: {telemetry['temp_c']} °C")
                response_received = True
                break

    if not response_received:
        print("No response from Motor ID 1 within 1 second.")
        print("Checking bus traffic for any active ID...")
        # Check if motor has a different ID
        for check_id in range(1, 10):
            bus.send(can.Message(arbitration_id=check_id, data=rest_data, is_extended_id=False))
            resp = bus.recv(timeout=0.05)
            if resp:
                print(f"Found active motor at CAN ID: {resp.arbitration_id} (0x{resp.arbitration_id:X})")
                print(f"Data: {resp.data.hex(' ')}")
                response_received = True
                break

    bus.shutdown()

if __name__ == "__main__":
    main()
