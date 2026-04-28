import tkinter as tk
from tkinter import ttk, messagebox
import serial
import serial.tools.list_ports
import threading
import collections
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import time
import numpy as np
import sys

# --- KONFIGURACJA ---
UPDATE_MS = 20 

class BikeDataManager:
    def __init__(self):
        self.mode = 'simulation'
        self.is_running = False
        self.ser = None
        self.data = [0.0, 0.0, 0.0, 0.0] # angle, velocity, wheel_vel, fill
        
    @staticmethod
    def get_available_ports():
        return [p.device for p in serial.tools.list_ports.comports()]

    def connect(self, port, baudrate=115200):
        try:
            self.ser = serial.Serial(port, baudrate, timeout=0.05)
            self.mode = 'hardware'
            self.is_running = True
            return True
        except Exception as e:
            messagebox.showerror("Serial Error", f"Błąd połączenia: {e}")
            return False

    def disconnect(self):
        self.is_running = False
        if self.ser and self.ser.is_open:
            self.send_command("stop")
            self.ser.close()
        self.mode = 'simulation'

    def send_command(self, cmd):
        if self.ser and self.ser.is_open:
            self.ser.write(f"{cmd}\n".encode('ascii'))

    def update_loop(self, callback):
        last_time = time.time()
        while self.is_running:
            now = time.time()
            dt = now - last_time
            last_time = now

            if self.mode == 'hardware' and self.ser:
                if self.ser.in_waiting:
                    line = self.ser.readline().decode('ascii', errors='replace').strip()
                    try:
                        vals = [float(x) for x in line.split(',')]
                        if len(vals) == 4:
                            self.data = vals
                            callback(self.data) # Przekazanie do konsoli i wykresu
                    except ValueError: pass
            else:
                angle, vel, w_vel, fill = self.data
                self.data = [angle + vel*dt, vel - 9.81*np.sin(angle)*dt, w_vel, fill]
                callback(self.data)
                time.sleep(0.01)

class BikeApp:
    def __init__(self, root):
        self.root = root
        self.root.title(f"Bike Monitor ({sys.platform})")
        self.manager = BikeDataManager()
        
        self.times = collections.deque(maxlen=500)
        self.angle_q = collections.deque(maxlen=500)
        self.wheel_q = collections.deque(maxlen=500)
        self.start_time = time.time()

        self.setup_ui()
        self.refresh_ui()

    def setup_ui(self):
        toolbar = ttk.Frame(self.root, padding="5")
        toolbar.pack(side=tk.TOP, fill=tk.X)

        ttk.Label(toolbar, text="Port:").pack(side=tk.LEFT)
        self.port_combo = ttk.Combobox(toolbar, values=self.manager.get_available_ports(), width=15)
        self.port_combo.pack(side=tk.LEFT, padx=5)
        
        self.btn_conn = ttk.Button(toolbar, text="Connect", command=self.toggle_connection)
        self.btn_conn.pack(side=tk.LEFT, padx=5)

        self.btn_run = ttk.Button(toolbar, text="START (PID)", command=lambda: self.manager.send_command("pid"), state=tk.DISABLED)
        self.btn_run.pack(side=tk.LEFT, padx=5)
        
        self.btn_stop = ttk.Button(toolbar, text="STOP", command=lambda: self.manager.send_command("stop"), state=tk.DISABLED)
        self.btn_stop.pack(side=tk.LEFT, padx=5)

        self.val_label = ttk.Label(toolbar, text="Angle: 0.000 rad", font=('Arial', 10, 'bold'))
        self.val_label.pack(side=tk.RIGHT, padx=10)

        self.fig, (self.ax1, self.ax2) = plt.subplots(2, 1, figsize=(8, 6))
        self.line_angle, = self.ax1.plot([], [], 'b-', label='Angle [rad]')
        self.line_wheel, = self.ax2.plot([], [], 'g-', label='Wheel [rad/s]')
        for ax in [self.ax1, self.ax2]: 
            ax.grid(True)
            ax.legend(loc='upper right')
        
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.root)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    def toggle_connection(self):
        if not self.manager.is_running:
            if self.manager.connect(self.port_combo.get()):
                self.btn_conn.config(text="Disconnect")
                self.btn_run.config(state=tk.NORMAL)
                self.btn_stop.config(state=tk.NORMAL)
                threading.Thread(target=self.manager.update_loop, args=(self.receive_data,), daemon=True).start()
        else:
            self.manager.disconnect()
            self.btn_conn.config(text="Connect")
            self.btn_run.config(state=tk.DISABLED)
            self.btn_stop.config(state=tk.DISABLED)

    # --- ODBIERANIE DANYCH Z WYŚWIETLANIEM W KONSOLI ---
    def receive_data(self, data):
        curr_time = time.time() - self.start_time
        self.times.append(curr_time)
        self.angle_q.append(data[0])
        self.wheel_q.append(data[2])
        
        # Wyświetlanie w konsoli
        print(f"Time: {curr_time:6.2f}s | Angle: {data[0]:7.4f} rad | Wheel: {data[2]:8.2f} rad/s | PWM: {data[3]:4.0f}")

    def refresh_ui(self):
        if len(self.times) > 0:
            t_list = list(self.times)
            angle, _, _, fill = self.manager.data
            
            self.line_angle.set_data(t_list, list(self.angle_q))
            self.line_wheel.set_data(t_list, list(self.wheel_q))
            
            for ax in [self.ax1, self.ax2]:
                ax.relim()
                ax.autoscale_view()
                if len(self.times) > 1:
                    ax.set_xlim(self.times[0], self.times[-1])

            self.val_label.config(text=f"Angle: {angle:.3f} rad | PWM: {fill:.0f}")
            self.canvas.draw()
        
        self.root.after(UPDATE_MS, self.refresh_ui)

if __name__ == "__main__":
    root = tk.Tk()
    app = BikeApp(root)
    root.mainloop()