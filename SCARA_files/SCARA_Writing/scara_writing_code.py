import math
import tkinter as tk
from typing import List, Union, Tuple
import serial
import time
from pynput.mouse import Listener

# ================================================================
# --- SCARA + Serial Config ---
# ================================================================
SERIAL_PORT = "COM4"   # <-- Change if needed
BAUDRATE = 115200
PEN_UP = 0
PEN_DOWN = 180

CM_TO_PX = 10
WORKSPACE_CM = 30
CANVAS_SIZE = WORKSPACE_CM * CM_TO_PX
L1, L2 = 10.0, 13.0


def solve_ik_lists(
    x_in: List[float],
    y_in: List[float],
    L1: float = 10.0,
    L2: float = 13.0,
    to_degrees: bool = True,
    tol: float = 1e-9
) -> Tuple[List[Union[float, None]], List[Union[float, None]]]:
    if len(x_in) != len(y_in):
        raise ValueError("x and y lists must have same length.")

    reachMax = L1 + L2
    reachMin = abs(L1 - L2)

    theta1 = []
    theta2 = []

    for x, y in zip(x_in, y_in):
        r = math.hypot(x, y)

        if (r > reachMax + tol) or (r < reachMin - tol):
            theta1.append(None)
            theta2.append(None)
            continue

        c2 = (x*x + y*y - L1*L1 - L2*L2) / (2.0 * L1 * L2)
        c2 = max(-1.0, min(1.0, c2))

        try:
            theta2_rad = math.acos(c2)
        except ValueError:
            theta1.append(None)
            theta2.append(None)
            continue

        numerator = L2 * math.sin(theta2_rad)
        denominator = L1 + L2 * c2
        theta1_rad = math.atan2(y, x) - math.atan2(numerator, denominator)

        if to_degrees:
            theta1_val = math.degrees(theta1_rad)
            theta2_val = math.degrees(theta2_rad)
        else:
            theta1_val = theta1_rad
            theta2_val = theta2_rad

        theta1_val = theta1_val % 360
        theta2_val = theta2_val+90 % 360

        if not (0 <= theta1_val <= 180 and 0 <= theta2_val <= 180):
            theta1.append(None)
            theta2.append(None)
            continue

        theta1.append(theta1_val)
        theta2.append(theta2_val)

    return theta1, theta2


# ================================================================
# --- Tkinter Drawing App ---
# ================================================================
class DrawingApp:
    def on_close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("🔌 Serial connection closed.")
        self.root.destroy()

    def __init__(self, root):
        self.root = root
        self.root.title("SCARA Workspace Drawing + IK with Serial Execution")

        self.canvas = tk.Canvas(root, width=CANVAS_SIZE, height=CANVAS_SIZE, bg="white")
        self.canvas.pack()

        for i in range(0, CANVAS_SIZE+1, 50):
            self.canvas.create_line(i, 0, i, CANVAS_SIZE, fill="#ddd")
            self.canvas.create_line(0, i, CANVAS_SIZE, i, fill="#ddd")

        self.draw_workspace_boundary()

        self.drawing = False
        self.last_x = None
        self.last_y = None
        self.x_list = []
        self.y_list = []
        self.count=0
        self.t1_l=0
        self.t2_l=0

        self.canvas.bind("<Button-1>", self.start_draw)
        self.canvas.bind("<B1-Motion>", self.draw)
        self.canvas.bind("<ButtonRelease-1>", self.stop_draw)

        self.solve_btn = tk.Button(root, text="Solve IK + Run SCARA", command=self.solve_ik_and_send)
        self.solve_btn.pack(pady=10)

        # Serial setup
        try:
            self.ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1)
            time.sleep(2)
            print("Connected to serial.")
        except Exception as e:
            print("⚠️ Serial connection failed:", e)
            self.ser = None

    def draw_workspace_boundary(self):
        base_x, base_y = 0, CANVAS_SIZE
        cx, cy = base_x, base_y

        outer_r = (L1 + L2) * CM_TO_PX
        #inner_r = abs(L1 - L2) * CM_TO_PX
        inner_r = 150

        self.canvas.create_arc(
            cx - outer_r, cy - outer_r, cx + outer_r, cy + outer_r,
            start=0, extent=180, outline="red", style="arc", width=3
        )
        self.canvas.create_arc(
            cx - inner_r, cy - inner_r, cx + inner_r, cy + inner_r,
            start=0, extent=180, outline="red", style="arc", width=3
        )

    def start_draw(self, event):
        self.drawing = True
        self.last_x = event.x
        self.last_y = event.y
        self.add_point(event.x, event.y)

    def draw(self, event):
        if self.drawing:
            self.canvas.create_line(self.last_x, self.last_y, event.x, event.y, fill="black")
            self.last_x, self.last_y = event.x, event.y
            self.add_point(event.x, event.y)

    def stop_draw(self, event):
        self.drawing = False
        self.x_list.append(13.0)
        self.y_list.append(0.0)
    
    def add_point(self, x_px, y_px):
        x_cm = x_px / CM_TO_PX
        y_cm = (CANVAS_SIZE - y_px) / CM_TO_PX
        self.x_list.append(x_cm)
        self.y_list.append(y_cm)

    def send_angles_to_scara(self, t1_list, t2_list):
        if not self.ser:
            print("⚠️ Cannot send: No serial connection.")
            return

        # Raise pen before moving
        # self.ser.write(f"-,-,{PEN_UP}\n".encode())
        # time.sleep(0.5)

        # Lower pen to start drawing
        # self.ser.write(f"-,-,{PEN_DOWN}\n".encode())
        # time.sleep(0.5)

        for t1, t2 in zip(t1_list, t2_list):
            if t1 is None or t2 is None:
                print("Skipping unreachable point.")
                msg = f"{int(self.t1_l)},{self.t2_l},{PEN_UP}\n"
                self.count+=1
                print(f"Sending: {msg.strip()}")
                self.ser.write(msg.encode())
                time.sleep(0.2)
                self.count=0
                continue
            else:
                if (self.count<3):
                    msg = f"{int(t1)},{int(t2)},{PEN_UP}\n"
                    self.count+=1
                    print(f"Sending: {msg.strip()}")
                    self.ser.write(msg.encode())
                    time.sleep(0.2)
                else:
                    msg = f"{int(t1)},{int(t2)},{PEN_DOWN}\n"
                    print(f"Sending: {msg.strip()}")
                    self.ser.write(msg.encode())
                    self.t1_l=t1
                    self.t2_l=t2
                    time.sleep(0.2)

        # Raise pen after drawing
        self.ser.write(f"{int(self.t1_l)},{self.t2_l},{PEN_UP}\n".encode())
        print("✅ Finished sending angles.")
        self.count=0

    def solve_ik_and_send(self):
        if not self.x_list:
            print("⚠️ No points drawn.")
            return

        t1, t2 = solve_ik_lists(self.x_list, self.y_list, L1, L2)

        t1_str = ", ".join("Unreachable" if v is None else f"{v:.2f}" for v in t1)
        t2_str = ", ".join("Unreachable" if v is None else f"{v:.2f}" for v in t2)

        print(f"theta1 = [{t1_str}]")
        print(f"theta2 = [{t2_str}]")

        self.send_angles_to_scara(t1, t2)

        # Reset path
        self.x_list = []
        self.y_list = []
        self.canvas.delete("all")
        self.draw_workspace_boundary()



if __name__ == "__main__":
    root = tk.Tk()
    app = DrawingApp(root)
    root.protocol("WM_DELETE_WINDOW", app.on_close)
    root.mainloop()
