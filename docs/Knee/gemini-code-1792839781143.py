import can
import time

def float_to_uint(val: float, val_min: float, val_max: float, bits: int) -> int:
    val = max(min(val, val_max), val_min)
    span = val_max - val_min
    return int(((val - val_min) / span) * ((1 << bits) - 1))

# Initialize PCAN interface (assuming PCAN_USBBUS1 at 1Mbps)
bus = can.interface.Bus(bustype='pcan', channel='PCAN_USBBUS1', bitrate=1000000)

motor_id = 2

# 1. Enter Rest State (Clear faults)
rest_msg = can.Message(arbitration_id=motor_id, data=[0xFF]*7 + [0xFD], is_extended_id=False)
bus.send(rest_msg)
time.sleep(0.1)

# 2. Enter Motor State (Enable power output)
motor_msg = can.Message(arbitration_id=motor_id, data=[0xFF]*7 + [0xFC], is_extended_id=False)
bus.send(motor_msg)
print("Motor enabled. Moving knee...")

# 3. Stream trajectory target (e.g., move to 1.0 radian ~ 57 degrees)
target_rad = 1.0
max_vel = 2.0

p_int = float_to_uint(target_rad, -12.5, 12.5, 16)
v_int = float_to_uint(max_vel, 0.0, 10.0, 8)

# Standard tuning bytes for position tracking
payload = [
    (p_int >> 8) & 0xFF,
    p_int & 0xFF,
    v_int,
    0x19,  # Pos Kp (~15)
    0x17,  # Pos Kd (~4.5)
    0x33,  # Vel Kp (~50)
    0x00,  # Vel Kd (0)
    0x05   # Vel Ki (~0.001)
]

try:
    for _ in range(100): # Stream for a couple of seconds
        msg = can.Message(arbitration_id=motor_id, data=payload, is_extended_id=False)
        bus.send(msg)
        
        # Read back telemetry response if available
        response = bus.recv(timeout=0.01)
        if response and response.arbitration_id == motor_id:
            status = response.data[0]
            fault = response.data[7]
            print(f"Status: {status:#04x} | Fault Code: {fault}")
            
        time.sleep(0.02) # 50Hz update rate
        
except KeyboardInterrupt:
    pass

# 4. Return to Rest State on exit
bus.send(rest_msg)
bus.shutdown()
print("Test completed. Motor safely rested.")