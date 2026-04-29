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
import numpy as np
import sys
import asyncio
from bleak import BleakClient, BleakScanner

# --- KONFIGURACJA ---
UPDATE_MS = 33
BLE_SERVICE_UUID = "19B10000-E8F2-537E-4F6C-D104768A1214"
BLE_DATA_UUID = "19B10001-E8F2-537E-4F6C-D104768A1214"


class BikeDataManager:
    def __init__(self):
        self.mode = 'simulation'
        self.is_running = False
        self.conn = None
        self.data = [0.0, 0.0, 0.0, 0.0]
        self.ble_loop = None

    @staticmethod
    def get_available_ports():
        return [p.device for p in serial.tools.list_ports.comports()]

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
            messagebox.showerror("WiFi Error", f"Błąd: {e}")
            return False

    async def connect_ble_task(self, callback):
        try:
            # ZAPISUJEMY aktualną pętlę, w której uruchomił się Bluetooth
            self.ble_loop = asyncio.get_running_loop()

            devices = await BleakScanner.discover()
            target = next((d for d in devices if d.name == "Bike-Project"), None)

            if not target:
                self.is_running = False
                return

            async with BleakClient(target.address) as client:
                self.conn = client  # To jest nasz obiekt BleakClient
                self.mode = 'hardware_ble'
                self.is_running = True

                def handler(sender, data):
                    try:
                        vals = [float(x) for x in data.decode().split(',')]
                        if len(vals) == 4:
                            self.data = vals
                            callback(self.data)
                    except:
                        pass

                await client.start_notify(BLE_DATA_UUID, handler)

                # Czekamy, dopóki połączenie ma trwać
                while self.is_running:
                    await asyncio.sleep(0.1)

                await client.stop_notify(BLE_DATA_UUID)
        except Exception as e:
            print(f"Błąd w zadaniu BLE: {e}")
            self.is_running = False
        finally:
            self.conn = None
            self.ble_loop = None

    def disconnect(self):
        self.is_running = False
        if self.conn:
            try:
                self.send_command("stop"); self.conn.close()
            except:
                pass
        self.conn = None
        self.mode = 'simulation'

    def send_command(self, cmd):
        """Wysyła komendy przez Serial, WiFi lub Bluetooth"""
        msg = f"{cmd}\n".encode('ascii')
        try:
            if self.mode == 'hardware_serial' and self.conn:
                self.conn.write(msg)
            elif self.mode == 'hardware_wifi' and self.conn:
                self.conn.send(msg)
            elif self.mode == 'hardware_ble' and self.conn and self.ble_loop:
                # Używamy zachowanej pętli (ble_loop), aby bezpiecznie wysłać
                # komendę z wątku Tkintera do wątku Bluetooth
                asyncio.run_coroutine_threadsafe(
                    self.conn.write_gatt_char(BLE_DATA_UUID, cmd.encode('ascii')),
                    self.ble_loop
                )
        except Exception as e:
            print(f"Błąd wysyłania komendy ({self.mode}): {e}")

    def update_loop(self, callback):
        while self.is_running:
            try:
                if self.mode == 'hardware_serial' and self.conn.in_waiting:
                    line = self.conn.readline().decode('ascii', errors='replace').strip()
                elif self.mode == 'hardware_wifi':
                    line = self.conn.recv(1024).decode('ascii', errors='replace').split('\n')[0].strip()
                else:
                    line = None
                if line:
                    vals = [float(x) for x in line.split(',')]
                    if len(vals) == 4: self.data = vals; callback(self.data)
            except:
                pass
            if self.mode == 'simulation':
                a, v, w, f = self.data
                self.data = [a + v * 0.01, v - 9.81 * np.sin(a) * 0.01, w, f]
                callback(self.data)
                time.sleep(0.01)


class BikeApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Bike Monitor - Pro Layout")
        self.manager = BikeDataManager()
        self._after_id = None

        self.full_history = 6000
        self.display_points = tk.IntVar(value=1000)
        self.show_visual = tk.BooleanVar(value=True)

        self.times = collections.deque(maxlen=self.full_history)
        self.angle_q = collections.deque(maxlen=self.full_history)
        self.wheel_q = collections.deque(maxlen=self.full_history)
        self.start_time = time.time()

        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        self.setup_ui()
        self.update_plot_layout()
        self.refresh_ui()

    def setup_ui(self):
        toolbar = ttk.Frame(self.root, padding="5")
        toolbar.pack(side=tk.TOP, fill=tk.X)

        self.conn_type = tk.StringVar(value="Serial")
        self.mode_menu = ttk.OptionMenu(toolbar, self.conn_type, "Serial", "Serial", "WiFi", "Bluetooth",
                                        command=self.on_mode_change)
        self.mode_menu.pack(side=tk.LEFT, padx=5)

        self.port_combo = ttk.Combobox(toolbar, values=self.manager.get_available_ports(), width=12)
        self.port_combo.pack(side=tk.LEFT, padx=2)

        self.ip_entry = ttk.Entry(toolbar, width=12)
        self.ip_entry.insert(0, "192.168.4.1")

        self.btn_conn = ttk.Button(toolbar, text="Connect", command=self.toggle_connection)
        self.btn_conn.pack(side=tk.LEFT, padx=2)

        ttk.Button(toolbar, text="START", command=lambda: self.manager.send_command("pid")).pack(side=tk.LEFT, padx=2)
        ttk.Button(toolbar, text="STOP", command=lambda: self.manager.send_command("stop")).pack(side=tk.LEFT, padx=2)

        self.history_slider = ttk.Scale(toolbar, from_=1, to=30, orient=tk.HORIZONTAL, command=self.update_history_size, length=80)
        self.history_slider.set(10)
        self.history_slider.pack(side=tk.LEFT, padx=5)

        self.lbl_hist = ttk.Label(toolbar, text="10s")
        self.lbl_hist.pack(side=tk.LEFT)

        ttk.Checkbutton(toolbar, text="Visual", variable=self.show_visual, command=self.update_plot_layout).pack(
            side=tk.LEFT, padx=5)

        self.val_label = ttk.Label(toolbar, text="0.000 rad | 0.0°", font=('Arial', 10, 'bold'))
        self.val_label.pack(side=tk.RIGHT, padx=10)

        self.fig = plt.figure(figsize=(11, 6))
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.root)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    def update_plot_layout(self):
        """Precyzyjne proporcje: Wykresy (66%) : Wizualizacja (33%)"""
        self.fig.clear()

        if self.show_visual.get():
            # GridSpec z width_ratios ustawia proporcje kolumn
            gs = self.fig.add_gridspec(2, 2, width_ratios=[3, 1])

            self.ax1 = self.fig.add_subplot(gs[0, 0])
            self.ax2 = self.fig.add_subplot(gs[1, 0])
            self.ax_vis = self.fig.add_subplot(gs[:, 1])

            self.ax_vis.set_aspect('equal')
            self.ax_vis.axis('off')
            self.ax_vis.set_xlim(-1.1, 1.1)
            self.ax_vis.set_ylim(-0.1, 1.2)
            self.ax_vis.axhline(0, color='black', lw=2)
            self.ax_vis.plot([0, np.sin(-0.4)], [0, np.cos(-0.4)], color='gray', ls='--', lw=1)
            self.ax_vis.plot([0, np.sin(0.4)], [0, np.cos(0.4)], color='gray', ls='--', lw=1)
            self.bike_arrow, = self.ax_vis.plot([0, 0], [0, 1], color='red', lw=4, solid_capstyle='round')
        else:
            gs = self.fig.add_gridspec(2, 1)
            self.ax1 = self.fig.add_subplot(gs[0, 0])
            self.ax2 = self.fig.add_subplot(gs[1, 0])
            self.ax_vis = None

        self.line_angle, = self.ax1.plot([], [], 'b-', lw=1, label='Angle')
        self.line_wheel, = self.ax2.plot([], [], 'g-', lw=1, label='Wheel')

        for ax in [self.ax1, self.ax2]:
            ax.grid(True, alpha=0.3)
            ax.legend(loc='upper right', fontsize='small')

        self.fig.tight_layout()
        self.canvas.draw_idle()

    def update_history_size(self, val):
        self.display_points.set(int(float(val) * 100))
        if hasattr(self, 'lbl_hist'): self.lbl_hist.config(text=f"{int(float(val))}s")

    def on_mode_change(self, val):
        self.port_combo.pack_forget()
        self.ip_entry.pack_forget()
        if val == "Serial":
            self.port_combo.pack(side=tk.LEFT, padx=2, after=self.mode_menu)
        elif val == "WiFi":
            self.ip_entry.pack(side=tk.LEFT, padx=2, after=self.mode_menu)

    def toggle_connection(self):
        if not self.manager.is_running:
            mode = self.conn_type.get()
            success = False
            if mode == "Serial":
                success = self.manager.connect_serial(self.port_combo.get())
            elif mode == "WiFi":
                success = self.manager.connect_wifi(self.ip_entry.get())
            elif mode == "Bluetooth":
                success = True
                threading.Thread(target=lambda: asyncio.run(self.manager.connect_ble_task(self.receive_data)),
                                 daemon=True).start()
            if success:
                self.btn_conn.config(text="Disconnect")
                if mode != "Bluetooth": threading.Thread(target=self.manager.update_loop, args=(self.receive_data,),
                                                         daemon=True).start()
        else:
            self.manager.disconnect(); self.btn_conn.config(text="Connect")

    def receive_data(self, data):
        curr_time = time.time() - self.start_time
        self.times.append(curr_time)
        self.angle_q.append(data[0])
        self.wheel_q.append(data[2])

    def refresh_ui(self):
        if len(self.times) > 0:
            num_pts = self.display_points.get()
            t_list = list(self.times)[-num_pts:]
            a_list = list(self.angle_q)[-num_pts:]
            w_list = list(self.wheel_q)[-num_pts:]

            self.line_angle.set_data(t_list, a_list)
            self.line_wheel.set_data(t_list, w_list)

            for ax in [self.ax1, self.ax2]:
                ax.relim()
                ax.autoscale_view()
                if len(t_list) > 1: ax.set_xlim(t_list[0], t_list[-1])

            if self.ax_vis:
                self.bike_arrow.set_data([0, np.sin(self.manager.data[0])], [0, np.cos(self.manager.data[0])])

            self.val_label.config(
                text=f"{self.manager.data[0]:.3f} rad | {np.degrees(self.manager.data[0]):.1f}° | PWM: {self.manager.data[3]:.0f}")
            self.canvas.draw_idle()

        self._after_id = self.root.after(UPDATE_MS, self.refresh_ui)

    def on_closing(self):
        self.manager.disconnect()
        if self._after_id: self.root.after_cancel(self._after_id)
        self.root.quit()
        self.root.destroy()


if __name__ == "__main__":
    root = tk.Tk()
    app = BikeApp(root)
    root.mainloop()