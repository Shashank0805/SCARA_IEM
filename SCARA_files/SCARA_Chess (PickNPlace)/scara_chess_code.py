import math
import tkinter as tk
from tkinter import messagebox
import serial
import time
import chess

# ================================================================
# --- ROBOT & PHYSICAL CONFIGURATION ---
# ================================================================
SERIAL_PORT = "COM11"  # <-- CHANGE THIS
BAUDRATE = 115200

# Arm Lengths (cm)
L1 = 12.0
L2 = 14.0

# Servo Angles (Adjust to your hardware calibration)
Z_UP = 0        # Servo angle for Arm High (Travel)
Z_DOWN = 180    # Servo angle for Arm Low (Grab/Drop)
GRIP_OPEN = 0  # Servo angle to Open
GRIP_CLOSE = 180 # Servo angle to Grab

# Workspace / Board Definition (CM)
# The board must fit in the "donut" shape of the SCARA workspace.
# We offset the board so (0,0) of the board is at (BOARD_OFFSET_X, BOARD_OFFSET_Y) relative to robot base.
BOARD_OFFSET_X = -6.0 
BOARD_OFFSET_Y = 10.0 
SQUARE_SIZE_CM = 1  # Size of one chess square in cm

# Graveyard (Where captured pieces go) - ensure this is reachable!
GRAVEYARD_X = -10.0
GRAVEYARD_Y = 10.0

# Visualization Scaling
PX_PER_CM = 30  # Zoom level for screen

# ================================================================
# --- INVERSE KINEMATICS ---
# ================================================================
def solve_scara_ik(x, y):
    """
    Returns (theta1, theta2) in degrees.
    Returns (None, None) if unreachable.
    """
    r = math.hypot(x, y)
    
    # Check boundaries
    if r > (L1 + L2) or r < abs(L1 - L2):
        return None, None

    # Law of Cosines for Elbow (Theta2)
    try:
        c2 = (x*x + y*y - L1*L1 - L2*L2) / (2.0 * L1 * L2)
        # Clamp for floating point noise
        c2 = max(-1.0, min(1.0, c2))
        theta2_rad = math.acos(c2) # Elbow up/down choice, usually + is fine for SCARA

        # Calculate Theta1
        k1 = L1 + L2 * math.cos(theta2_rad)
        k2 = L2 * math.sin(theta2_rad)
        theta1_rad = math.atan2(y, x) - math.atan2(k2, k1)

        t1 = math.degrees(theta1_rad)
        t2 = math.degrees(theta2_rad)

        # Normalize logic (Adjust based on your specific servo mounting)
        # Assuming standard setup where 0 is right, 90 is forward
        # You might need t1 = t1 % 180 or t2 = 180 - t2 depending on hardware
        
        # Simple constraint check (0-180 servos)
        # if 0 <= t1 <= 180 and 0 <= t2 <= 180:
        return t1, t2
        
    except ValueError:
        return None, None

# ================================================================
# --- ROBOT CONTROLLER CLASS ---
# ================================================================
class ScaraRobot:
    def __init__(self):
        try:
            self.ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1)
            time.sleep(2) # Wait for Arduino reset
            print(f"🔌 Connected to Robot on {SERIAL_PORT}")
        except Exception as e:
            print(f"⚠️ Robot not connected: {e}")
            self.ser = None

    def send_command(self, t1, t2, z, g):
        """
        Protocol: T1,T2,Z,G\n
        Integers sent to Arduino.
        """
        if self.ser:
            msg = f"{int(t1)},{int(t2)},{int(z)},{int(g)}\n"
            self.ser.write(msg.encode())
            # print(f"Sent: {msg.strip()}")
            time.sleep(0.05) # Small buffer
        else:
            print(f"[SIM] T1:{int(t1)} T2:{int(t2)} Z:{z} G:{g}")

    def execute_move(self, start_cm, end_cm, is_capture=False):
        """
        Full pick and place sequence.
        start_cm: (x, y) tuple
        end_cm: (x, y) tuple
        """
        sx, sy = start_cm
        ex, ey = end_cm
        
        # 1. Calculate Angles
        st1, st2 = solve_scara_ik(sx, sy)
        et1, et2 = solve_scara_ik(ex, ey)

        if st1 is None or et1 is None:
            print("❌ Move Out of Workspace!")
            return False

        print(f"🤖 Moving from ({sx},{sy}) to ({ex},{ey})")

        # SEQUENCE:
        
        # 1. Go to Start, Z-Up, Open Gripper
        self.send_command(st1, st2, Z_UP, GRIP_OPEN)
        time.sleep(1.0) # Travel time

        # 2. Lower Z (Prepare to Grab)
        self.send_command(st1, st2, Z_DOWN, GRIP_OPEN)
        time.sleep(0.5)

        # 3. Close Gripper (Grab)
        self.send_command(st1, st2, Z_DOWN, GRIP_CLOSE)
        time.sleep(0.5)

        # 4. Raise Z (Lift)
        self.send_command(st1, st2, Z_UP, GRIP_CLOSE)
        time.sleep(0.5)

        # 5. Move to End (Carry)
        self.send_command(et1, et2, Z_UP, GRIP_CLOSE)
        time.sleep(1.0) # Travel time

        # 6. Lower Z (Prepare to Drop)
        self.send_command(et1, et2, Z_DOWN, GRIP_CLOSE)
        time.sleep(0.5)

        # 7. Open Gripper (Drop)
        self.send_command(et1, et2, Z_DOWN, GRIP_OPEN)
        time.sleep(0.5)

        # 8. Raise Z (Clear)
        self.send_command(et1, et2, Z_UP, GRIP_OPEN)
        time.sleep(0.5)

        return True

    def discard_piece(self, coords_cm):
        """Pick up a piece at coords and dump it in the graveyard."""
        print("🗑️ Discarding capture...")
        self.execute_move(coords_cm, (GRAVEYARD_X, GRAVEYARD_Y))


# ================================================================
# --- CHESS GUI ---
# ================================================================
class ChessApp:
    def __init__(self, root):
        self.root = root
        self.root.title("SCARA Chess Master")
        self.root.state('zoomed') # Full Screen

        self.robot = ScaraRobot()
        self.board = chess.Board()
        
        # UI State
        self.selected_square = None
        self.valid_moves = []
        
        # Calculate Canvas Dimensions
        self.canvas_width = 1000
        self.canvas_height = 800
        self.canvas = tk.Canvas(root, width=self.canvas_width, height=self.canvas_height, bg="#202020")
        self.canvas.pack(fill="both", expand=True)

        self.status_label = tk.Label(root, text="White to Move", font=("Arial", 16))
        self.status_label.pack()

        # Bindings
        self.canvas.bind("<Button-1>", self.on_click)
        self.root.bind("<Configure>", self.draw_board) # Redraw on resize

        self.draw_board()

    def get_square_center_cm(self, square_index):
        """
        Convert chess square index (0-63) to physical Robot Coordinates (cm).
        Rank 0 is bottom, Rank 7 is top. File 0 is left.
        """
        file_idx = chess.square_file(square_index) # 0-7 (X)
        rank_idx = chess.square_rank(square_index) # 0-7 (Y)

        # Calculate Physical Center
        phys_x = BOARD_OFFSET_X + (file_idx * SQUARE_SIZE_CM) + (SQUARE_SIZE_CM / 2)
        phys_y = BOARD_OFFSET_Y + (rank_idx * SQUARE_SIZE_CM) + (SQUARE_SIZE_CM / 2)
        return (phys_x, phys_y)

    def draw_board(self, event=None):
        self.canvas.delete("all")
        
        # Update canvas center
        cw = self.canvas.winfo_width()
        ch = self.canvas.winfo_height()
        if cw < 100: cw = 800 # Startup safety
        if ch < 100: ch = 800
        
        # Visual Constants for Screen (Px)
        SQ_PX = 80
        BOARD_PX_SIZE = SQ_PX * 8
        OFFSET_X = (cw - BOARD_PX_SIZE) // 2
        OFFSET_Y = (ch - BOARD_PX_SIZE) // 2

        # 1. Draw Squares
        for sq in range(64):
            file_idx = chess.square_file(sq)
            rank_idx = 7 - chess.square_rank(sq) # UI draws top-down
            
            x1 = OFFSET_X + file_idx * SQ_PX
            y1 = OFFSET_Y + rank_idx * SQ_PX
            x2 = x1 + SQ_PX
            y2 = y1 + SQ_PX
            
            color = "#DDB88C" if (file_idx + rank_idx) % 2 == 0 else "#A66D4F"
            
            # Highlight selected
            if sq == self.selected_square:
                color = "#60d394" # Green
            elif sq in self.valid_moves:
                color = "#f6d860" # Yellow hint

            self.canvas.create_rectangle(x1, y1, x2, y2, fill=color, outline="")
            
            # Store pixel coords for click detection
            self.canvas.addtag_withtag(f"sq_{sq}", self.canvas.create_rectangle(x1, y1, x2, y2, fill=color, outline=""))

            # 2. Draw Pieces
            piece = self.board.piece_at(sq)
            if piece:
                symbol = piece.unicode_symbol()
                # Determine color for font
                font_color = "black" if piece.color == chess.BLACK else "white"
                # Add shadow for white pieces to see them
                if piece.color == chess.WHITE:
                    self.canvas.create_text(x1 + SQ_PX/2 + 2, y1 + SQ_PX/2 + 2, text=symbol, font=("Arial", 40), fill="black")
                
                self.canvas.create_text(x1 + SQ_PX/2, y1 + SQ_PX/2, text=symbol, font=("Arial", 40), fill=font_color)

        # 3. Draw Workspace Boundary (Visual Guide)
        # Convert CM to Pixels relative to the board UI
        # This is tricky because the UI is centered, but robot is absolute.
        # We just sketch the robot Base relative to the board
        base_ui_x = OFFSET_X - (BOARD_OFFSET_X / SQUARE_SIZE_CM) * SQ_PX
        base_ui_y = (OFFSET_Y + 8*SQ_PX) + (BOARD_OFFSET_Y / SQUARE_SIZE_CM) * SQ_PX 
        # Note: Y is inverted in screen coords (Up is -Y) so this is approx.
        
        self.canvas.create_oval(base_ui_x-10, base_ui_y-10, base_ui_x+10, base_ui_y+10, fill="red")
        self.canvas.create_text(base_ui_x, base_ui_y+20, text="ROBOT BASE", fill="red")

    def on_click(self, event):
        # Determine square clicked
        cw = self.canvas.winfo_width()
        ch = self.canvas.winfo_height()
        SQ_PX = 80
        BOARD_PX_SIZE = SQ_PX * 8
        OFFSET_X = (cw - BOARD_PX_SIZE) // 2
        OFFSET_Y = (ch - BOARD_PX_SIZE) // 2

        col = (event.x - OFFSET_X) // SQ_PX
        row = (event.y - OFFSET_Y) // SQ_PX
        
        if not (0 <= col < 8 and 0 <= row < 8):
            return

        rank = 7 - row
        clicked_sq = chess.square(col, rank)

        # Logic
        if self.selected_square is None:
            # Selecting a piece
            piece = self.board.piece_at(clicked_sq)
            if piece and piece.color == self.board.turn:
                self.selected_square = clicked_sq
                self.valid_moves = [m.to_square for m in self.board.legal_moves if m.from_square == clicked_sq]
                self.draw_board()
        else:
            # Trying to move
            move = chess.Move(self.selected_square, clicked_sq)
            
            # Handle Promotion (Auto-promote to Queen for simplicity)
            if chess.square_rank(clicked_sq) in [0, 7] and self.board.piece_at(self.selected_square).piece_type == chess.PAWN:
                move.promotion = chess.QUEEN

            if move in self.board.legal_moves:
                self.perform_move(move)
            else:
                # Deselect or select new piece
                piece = self.board.piece_at(clicked_sq)
                if piece and piece.color == self.board.turn:
                    self.selected_square = clicked_sq
                    self.valid_moves = [m.to_square for m in self.board.legal_moves if m.from_square == clicked_sq]
                else:
                    self.selected_square = None
                    self.valid_moves = []
            self.draw_board()

    def perform_move(self, move):
        source_idx = move.from_square
        target_idx = move.to_square

        src_cm = self.get_square_center_cm(source_idx)
        dst_cm = self.get_square_center_cm(target_idx)

        # 1. Check for Capture
        captured_piece = self.board.piece_at(target_idx)
        if captured_piece:
            self.status_label.config(text="Processing Capture...", fg="red")
            self.root.update()
            # Robot: Move captured piece to Graveyard
            self.robot.discard_piece(dst_cm)
        
        # 2. Move Main Piece
        self.status_label.config(text="Robot Moving...", fg="blue")
        self.root.update()
        
        success = self.robot.execute_move(src_cm, dst_cm)
        
        if success:
            self.board.push(move)
            self.status_label.config(text="Move Complete. " + ("White" if self.board.turn else "Black") + " to move.", fg="black")
            
            # Check for Castling (The robot needs to move the Rook too!)
            # Using chess lib is_castling is tricky after the push.
            # Simple check: King moved > 1 square
            if abs(source_idx - target_idx) == 2 and self.board.piece_at(target_idx).piece_type == chess.KING:
                self.handle_castling_rook(source_idx, target_idx)
                
        else:
            self.status_label.config(text="Hardware Error: Unreachable", fg="red")

        self.selected_square = None
        self.valid_moves = []
        self.draw_board()

    def handle_castling_rook(self, k_src, k_dst):
        """Move the rook physically if castling occurred"""
        # Determine rook positions
        if k_dst > k_src: # Kingside
            r_src = k_dst + 1 # Corner
            r_dst = k_dst - 1 # Jump over king
        else: # Queenside
            r_src = k_dst - 2
            r_dst = k_dst + 1
            
        # Coordinates
        r_src_cm = self.get_square_center_cm(r_src)
        r_dst_cm = self.get_square_center_cm(r_dst)
        
        self.status_label.config(text="Adjusting Rook...", fg="blue")
        self.root.update()
        self.robot.execute_move(r_src_cm, r_dst_cm)


if __name__ == "__main__":
    root = tk.Tk()
    app = ChessApp(root)
    root.mainloop()