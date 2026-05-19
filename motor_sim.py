import socket
import time
import math
import threading

class BikeSimulator:
    def __init__(self):
        self.angle = 0.1  # Startowe wychylenie (rad)
        self.velocity = 0.0
        self.running = False
        self.dt = 0.033  # 33ms (~30 FPS)

    def update_physics(self):
        while True:
            if self.running:
                # Prosta fizyka: przyspieszenie grawitacyjne zwieksza kat
                # Grawitacja g = 9.81
                gravity_acc = 9.81 * math.sin(self.angle)
                self.velocity += gravity_acc * self.dt
                self.angle += self.velocity * self.dt
                
                # Jeśli "upadnie", resetujemy symulację
                if abs(self.angle) > 1.5:
                    self.angle = 0.1
                    self.velocity = 0.0
            time.sleep(self.dt)

    def start_server(self):
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.bind(('0.0.0.0', 80)) # Udajemy serwer Arduino
        server.listen(1)
        print("Symulator motorka czeka na połączenie na porcie 80...")

        while True:
            client, addr = server.accept()
            print(f"Połączono z apką: {addr}")
            try:
                while True:
                    # 1. Wysyłanie danych w formacie: angle,velocity,wheel_vel,pwm
                    data = f"{self.angle:.4f},{self.velocity:.4f},0.0,255\n"
                    client.send(data.encode('ascii'))
                    
                    # 2. Odbieranie komend (nieblokujące)
                    client.settimeout(0.01)
                    try:
                        cmd = client.recv(1024).decode().strip()
                        if cmd == "pid": 
                            self.running = True
                            print("Komenda: START")
                        if cmd == "stop": 
                            self.running = False
                            print("Komenda: STOP")
                    except socket.timeout:
                        pass
                    
                    time.sleep(self.dt)
            except Exception as e:
                print(f"Rozłączono: {e}")
                client.close()

sim = BikeSimulator()
threading.Thread(target=sim.update_physics, daemon=True).start()
sim.start_server()