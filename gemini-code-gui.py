import can
import math
import time
import threading
import tkinter as tk
from tkinter import ttk

class ProstheticKneeController:
    def __init__(self, root):
        self.root = root
        self.root.title("Prosthetic Knee Joint Controller (0° - 105°)")
        self.root.geometry("450x650")
        self.root.minsize(420, 600)

        # CAN Bus & State variables
        self.bus = None
        self.motor_id = 1
        self.is_connected = False
        self.running = False
        
        self.zero_offset_rad = 0.0
        self.target_deg = 0.0

        self.setup_ui()
        self.init_can_bus()

    def setup_ui(self):
        title_lbl = tk.Label(self.root, text="Motorevo Prosthetic Knee Studio", font=("Arial", 14, "bold"))
        title_lbl.pack(pady=10)

        # Connection Status Frame
        self.status_frame = tk.Frame(self.root, bg="red", height=30)
        self.status_frame.pack(fill=tk.X, padx=20, pady=5)
        self.status_lbl = tk.Label(self.status_frame, text="CAN Disconnected", bg="red", fg="white", font=("Arial", 10, "bold"))
        self.status_lbl.pack(pady=5)

        # Control Panel Frame
        control_frame = ttk.LabelFrame(self.root, text=" Joint Operation ")
        control_frame.pack(fill=tk.BOTH, expand=True, padx=20, pady=10)

        # Zero Position Button
        self.zero_btn = ttk.Button(control_frame, text="📍 Set Current Position as Zero (0°)", command=self.set_zero_position)
        self.zero_btn.pack(pady=15, ipadx=10, ipady=5)

        # Angle Knob / Scale (0 to 105 degrees)
        lbl_knob = ttk.Label(control_frame, text="Knee Flexion Angle (Degrees):", font=("Arial", 10))
        lbl_knob.pack(pady=(10, 0))

        # Readout label declared BEFORE the slider command references it
        self.angle_val_lbl = ttk.Label(control_frame, text="0.0° (0.00 rad)", font=("Arial", 12, "bold"), foreground="#0055ff")
        self.angle_val_lbl.pack(pady=5)

        self.angle_slider = ttk.Scale(control_frame, from_=0, to=105, orient=tk.HORIZONTAL, command=self.on_slider_change)
        self.angle_slider.set(0)
        self.angle_slider.pack(fill=tk.X, padx=30, pady=10)

        # Velocity & Gains Settings Frame
        gains_frame = ttk.LabelFrame(self.root, text=" Tuning & Limits ")
        gains_frame.pack(fill=tk.X, padx=20, pady=5)

        ttk.Label(gains_frame, text="Max Velocity (rad/s):").grid(row=0, column=0, sticky=tk.W, padx=10, pady=5)
        self.vel_spin = ttk.Spinbox(gains_frame, from_=0.5, to=10.0, increment=0.5, width=8)
        self.vel_spin.set(3.0)
        self.vel_spin.grid(row=0, column=1, padx=10, pady=5)

        ttk.Label(gains_frame, text="Position Kp:").grid(row=1, column=0, sticky=tk.W, padx=10, pady=5)
        self.kp_spin = ttk.Spinbox(gains_frame, from_=1.0, to=100.0, increment=1.0, width=8)
        self.kp_spin.set(20.0)
        self.kp_spin.grid(row=1, column=1, padx=10, pady=5)

        ttk.Label(gains_frame, text="Position Kd:").grid(row=2, column=0, sticky=tk.W, padx=10, pady=5)
        self.kd_spin = ttk.Spinbox(gains_frame, from_=0.0, to=20.0, increment=0.5, width=8)
        self.kd_spin.set(4.0)
        self.kd_spin.grid(row=2, column=1, padx=10, pady=5)

        # Telemetry Frame
        tele_frame = ttk.LabelFrame(self.root, text=" Real-Time Telemetry ")
        tele_frame.pack(fill=tk.X, padx=20, pady=10)

        self.tele_status_lbl = ttk.Label(tele_frame, text="Motor Status: Rest State", font=("Arial", 9))
        self.tele_status_lbl.pack(anchor=tk.W, padx=10, pady=2)

        self.tele_fault_lbl = ttk.Label(tele_frame, text="Fault Code: 0 (No Error)", font=("Arial", 9))
        self.tele_fault_lbl.pack(anchor=tk.W, padx=10, pady=2)

    def init_can_bus(self):
        try:
            # Initialize PCAN interface at 1Mbps
            self.bus = can.interface.Bus(interface='pcan', channel='PCAN_USBBUS1', bitrate=1000000)
            self.is_connected = True
            self.status_frame.config(bg="green")
            self.status_lbl.config(text="CAN Connected (PCAN-USB 1Mbps)", bg="green")
            
            # Send Enter Rest State, then Enter Motor State safely
            self.bus.send(can.Message(arbitration_id=self.motor_id, data=[0xFF]*7 + [0xFD], is_extended_id=False))
            time.sleep(0.05)
            self.bus.send(can.Message(arbitration_id=self.motor_id, data=[0xFF]*7 + [0xFC], is_extended_id=False))
            
            # Start background thread
            self.running = True
            self.thread = threading.Thread(target=self.can_worker, daemon=True)
            self.thread.start()
        except Exception as e:
            self.status_frame.config(bg="red")
            self.status_lbl.config(text=f"Connection Error: {str(e)}", bg="red")
            self.is_connected = False

    def on_slider_change(self, val):
        self.target_deg = float(val)
        rad = math.radians(self.target_deg)
        if hasattr(self, 'angle_val_lbl'):
            self.angle_val_lbl.config(text=f"{self.target_deg:.1f}° ({rad:.2f} rad)")

    def set_zero_position(self):
        try:
            if self.bus and self.is_connected:
                # 1. Send the Set Zero command (0xFE)
                zero_cmd = can.Message(arbitration_id=self.motor_id, data=[0xFF]*7 + [0xFE], is_extended_id=False)
                self.bus.send(zero_cmd)
                print("Machine zero position command sent.")
                
                # 2. Reset slider to 0 so target matches the new zero immediately (prevents jumping)
                self.angle_slider.set(0)
                self.target_deg = 0.0
                
                # 3. Brief pause for firmware to register and save offset
                time.sleep(0.15)
                
                # 4. Automatically re-enable Motor State so the LED lights back up
                enable_cmd = can.Message(arbitration_id=self.motor_id, data=[0xFF]*7 + [0xFC], is_extended_id=False)
                self.bus.send(enable_cmd)
                print("Motor re-enabled after setting zero.")
        except Exception as e:
            print(f"Error setting zero: {e}")
            
    def float_to_uint(self, val, val_min, val_max, bits):
        val = max(min(val, val_max), val_min)
        span = val_max - val_min
        if span == 0:
            return 0
        return int(((val - val_min) / span) * ((1 << bits) - 1))

    def can_worker(self):
        while self.running:
            if not self.is_connected:
                time.sleep(0.1)
                continue

            try:
                deg = float(self.angle_slider.get())
                target_rad = math.radians(deg) + self.zero_offset_rad
                
                max_vel = float(self.vel_spin.get())
                kp = float(self.kp_spin.get())
                kd = float(self.kd_spin.get())

                # Matched to MParameter Studio limits: Theta MIN = 0.0, Theta MAX = 1.83 (~105°)
                p_int = self.float_to_uint(target_rad, 0.0, 1.83, 16)
                v_int = self.float_to_uint(0.0, -10.0, 10.0, 12)       
                kp_int = self.float_to_uint(kp, 0.0, 250.0, 12)     
                kd_int = self.float_to_uint(kd, 0.0, 50.0, 12)       
                t_int = self.float_to_uint(0.0, -50.0, 50.0, 12)      
                
                byte0 = (p_int >> 8) & 0xFF
                byte1 = p_int & 0xFF
                byte2 = (v_int >> 4) & 0xFF
                byte3 = ((v_int & 0x0F) << 4) | ((kp_int >> 8) & 0x0F)
                byte4 = kp_int & 0xFF
                byte5 = (kd_int >> 4) & 0xFF
                byte6 = ((kd_int & 0x0F) << 4) | ((t_int >> 8) & 0x0F)
                byte7 = t_int & 0xFF
                
                payload = [byte0, byte1, byte2, byte3, byte4, byte5, byte6, byte7]
                
                self.bus.send(can.Message(arbitration_id=self.motor_id, data=payload, is_extended_id=False))
                
                # Non-blocking check for feedback
                resp = self.bus.recv(timeout=0.005)
                if resp and resp.arbitration_id == self.motor_id:
                    status = resp.data[0]
                    fault = resp.data[7]
                    status_str = "Motor State (Active)" if status == 0x02 else f"Rest/State ({status:#04x})"
                    self.root.after(0, lambda s=status_str, f=fault: self.update_telemetry(s, f))

            except can.CanOperationError:
                pass
            except Exception as ex:
                print(f"CAN transmit error: {ex}")

            time.sleep(0.02)

    def update_telemetry(self, status_str, fault_code):
        if not self.root.winfo_exists():
            return
        self.tele_status_lbl.config(text=f"Motor Status: {status_str}")
        self.tele_fault_lbl.config(text=f"Fault Code: {fault_code}")
        if fault_code != 0:
            self.tele_fault_lbl.config(foreground="red")
        else:
            self.tele_fault_lbl.config(foreground="black")

    def on_closing(self):
        self.running = False
        if self.bus and self.is_connected:
            try:
                self.bus.send(can.Message(arbitration_id=self.motor_id, data=[0xFF]*7 + [0xFD], is_extended_id=False))
                self.bus.shutdown()
            except:
                pass
        self.root.destroy()

if __name__ == "__main__":
    root = tk.Tk()
    app = ProstheticKneeController(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    root.mainloop()