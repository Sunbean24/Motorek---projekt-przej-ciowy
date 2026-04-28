import tkinter as tk
from tkinter import ttk, messagebox
import serial
import serial.tools.list_ports
import socket
import threading
import collections
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import time
import sys

# --- KONFIGURACJA ---
UPDATE_MS = 20 

class BikeDataManager:
    def __init__(self):
        self.mode = 'simulation' # hardware_serial, hardware_wifi, simulation
        self.is_running = False
        self.conn = None # Przechowuje Serial lub Socket
        self.data = [0.0, 0.0, 0.0, 0.0]
        
    def connect_serial(self, port, baudrate=115200):
        try:
            self.conn = serial.Serial(port, baudrate, timeout=0.05)
            self.mode = 'hardware_serial'
            self.is_running = True
            return True
        except Exception as e:
            messagebox.showerror("Serial Error", f"Błąd: {e}")
            return False

    def connect_wifi(self, ip, port=80):
        try:
            self.conn = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.conn.settimeout(2.0)
            self.conn.connect((ip, int(port)))
            self.mode = 'hardware_wifi'
            self.is_running = True
            return True
        except Exception as e:
            messagebox.showerror("WiFi Error", f"Błąd połączenia z {ip}: {e}")
            return False

    def disconnect(self):
        self.is_running = False
        if self.conn:
            try:
                if self.mode == 'hardware_serial': self.conn.close()
                else: self.conn.close()
            except: pass
        self.mode = 'simulation'

    def send_command(self, cmd):
        msg = f"{cmd}\n".encode('ascii')
        try:
            if self.mode == 'hardware_serial' and self.conn: self.conn.write(msg)
            elif self.mode == 'hardware_wifi' and self.conn: self.conn.send(msg)
        except: print("Błąd wysyłania komendy")

    def update_loop(self, callback):
        while self.is_running:
            line = None
            try:
                if self.mode == 'hardware_serial' and self.conn.in_waiting:
                    line = self.conn.readline().decode('ascii', errors='replace').strip()
                elif self.mode == 'hardware_wifi':
                    line = self.conn.recv(1024).decode('ascii', errors='replace').split('\n')[0].strip()
                
                if line:
                    vals = [float(x) for x in line.split(',')]
                    if len(vals) == 4:
                        self.data = vals
                        callback(self.data)
            except: pass
            if self.mode == 'simulation': time.sleep(0.01)

class BikeApp:
    def __init__(self, root):
        self.root = root
        self.root.title(f"Bike Wireless Monitor ({sys.platform})")
        self.manager = BikeDataManager()
        self.times, self.angle_q, self.wheel_q = collections.deque(maxlen=500), collections.deque(maxlen=500), collections.deque(maxlen=500)
        self.start_time = time.time()
        self.setup_ui()
        self.refresh_ui()

    def setup_ui(self):
        toolbar = ttk.Frame(self.root, padding="5")
        toolbar.pack(side=tk.TOP, fill=tk.X)

        # Wybór medium
        self.conn_type = tk.StringVar(value="Serial")
        ttk.OptionMenu(toolbar, self.conn_type, "Serial", "Serial", "WiFi").pack(side=tk.LEFT, padx=5)
        
        self.ent_addr = ttk.Entry(toolbar, width=15) # Tu wpisujesz COM lub IP
        self.ent_addr.insert(0, "COM3")
        self.ent_addr.pack(side=tk.LEFT, padx=5)
        
        ttk.Button(toolbar, text="Connect", command=self.toggle_connection).pack(side=tk.LEFT)
        ttk.Button(toolbar, text="START", command=lambda: self.manager.send_command("pid")).pack(side=tk.LEFT, padx=2)
        ttk.Button(toolbar, text="STOP", command=lambda: self.manager.send_command("stop")).pack(side=tk.LEFT)

        self.fig, (self.ax1, self.ax2) = plt.subplots(2, 1, figsize=(8, 6))
        self.line_angle, = self.ax1.plot([], [], 'b-', label='Angle [rad]')
        self.line_wheel, = self.ax2.plot([], [], 'g-', label='Wheel [rad/s]')
        for ax in [self.ax1, self.ax2]: ax.grid(True); ax.legend(loc='upper right')
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.root)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    def toggle_connection(self):
        if not self.manager.is_running:
            addr = self.ent_addr.get()
            success = self.manager.connect_serial(addr) if self.conn_type.get() == "Serial" else self.manager.connect_wifi(addr)
            if success: threading.Thread(target=self.manager.update_loop, args=(self.receive_data,), daemon=True).start()
        else: self.manager.disconnect()

    def receive_data(self, data):
        curr_time = time.time() - self.start_time
        self.times.append(curr_time)
        self.angle_q.append(data[0])
        self.wheel_q.append(data[2])
        print(f"[{self.conn_type.get()}] Angle: {data[0]:.4f} | PWM: {data[3]:.0f}")

    def refresh_ui(self):
        if self.times:
            self.line_angle.set_data(list(self.times), list(self.angle_q))
            self.line_wheel.set_data(list(self.times), list(self.wheel_q))
            for ax in [self.ax1, self.ax2]:
                ax.relim(); ax.autoscale_view()
                if len(self.times) > 1: ax.set_xlim(self.times[0], self.times[-1])
            self.canvas.draw()
        self.root.after(UPDATE_MS, self.refresh_ui)

if __name__ == "__main__":
    root = tk.Tk(); app = BikeApp(root); root.mainloop()