#include <ArduinoBLE.h>
#include "DCMotor.h"
#include <ArduinoMotorCarrier.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Arduino.h>
#include <WiFiNINA.h>
#include "friction.h"
#include "filter.h"

#define enc_to_rad 0.1308996938995
#define DEG_TO_RAD 0.01745329 // pi/180

volatile float I = 0, Ts=0.01;
volatile int Ts_micro = 10000;
volatile float kp=4000.0f, ki=30.0f, kd=500.0f, k_omega = -0.007;
volatile float integral_decay_factor = 0.9999f;
volatile float angle=0.0f, velocity=0.0f, acc=0.0f;
volatile int lastMicros = 0, lastCount;

Adafruit_BNO055 bno = Adafruit_BNO055(55);
bool runPID = true;

// Charakterystyka BLE dla aplikacji mobilnej
BLEService bikeService("19B10000-E8F2-537E-4F6C-D104768A1214");
BLEStringCharacteristic dataChar("19B10001-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify, 64);

// Bluetooth
void setup_ble() {
  if (!BLE.begin()) return;
  BLE.setLocalName("Bike-Project");
  BLE.setAdvertisedService(bikeService);
  bikeService.addCharacteristic(dataChar);
  BLE.addService(bikeService);
  BLE.advertise();
}

// --- KONFIGURACJA WIFI ---
char ssid[] = "Motorek_Projekt"; // Nazwa sieci WiFi widoczna w laptopie
char pass[] = "inzynierka2026";  // Hasło (min. 8 znaków)
WiFiServer server(80);
WiFiClient remoteClient;         // Globalny obiekt klienta dla WiFi

void setup_wifi() {
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Błąd: Brak modułu WiFi!");
    return;
  }
  
  // Tworzenie własnej sieci (Access Point)
  Serial.print("Tworzenie sieci AP: ");
  Serial.println(ssid);
  
  if (WiFi.beginAP(ssid, pass) != WL_AP_LISTENING) {
    Serial.println("Błąd tworzenia punktu dostępowego!");
    while (1);
  }
  
  server.begin();
  Serial.print("IP motorka: "); 
  Serial.println(WiFi.localIP()); // Zazwyczaj 192.168.4.1
}

// Funkcja pomocnicza do obsługi komend (Serial i WiFi)
void handleCommand(String command) {
  command.trim();
  if (command.startsWith("setkp ")) {
    kp = command.substring(6).toFloat();
    Serial.print("kp set to: "); Serial.println(kp, 6);
  } else if (command.startsWith("setki ")) {
    ki = command.substring(6).toFloat();
    Serial.print("ki set to: "); Serial.println(ki, 6);
  } else if (command.startsWith("setkd ")) {
    kd = command.substring(6).toFloat();
    Serial.print("kd set to: "); Serial.println(kd, 6);
  } else if (command.startsWith("setkomega ")) {
    k_omega = command.substring(10).toFloat();
    Serial.print("k_omega set to: "); Serial.println(k_omega, 6);
  } else if (command.equals("pid")) {
    runPID = true;
    I = 0;
    friction_correction_integral = 0;
    Serial.println("Running PID controller");
  } else if (command.equals("stop")) {
    runPID = false;
    M2.setDuty(0);
    M3.setDuty(0);
    Serial.println("Stopping / Battery mode");
  }
}

void setup() {
  Serial.begin(115200);
  
  setup_wifi(); // Inicjalizacja WiFi

  if (!bno.begin()) {
    Serial.println("Failed to initialize BNO055!");
    while (1);
  }
  
  delay(1000);
  bno.setExtCrystalUse(true);
  bno.setMode(adafruit_bno055_opmode_t::OPERATION_MODE_NDOF);
  
  encoder1.resetCounter(0);
  encoder2.resetCounter(0);
  controller.begin();
  
  M1.setDuty(0); M2.setDuty(0); M3.setDuty(0); M4.setDuty(0);
  servo2.setAngle(100);
  delay(100);
  servo2.detach();
}

void loop() {
  // 1. Obsługa komend z Serial (USB)
  if (Serial.available()) {
    handleCommand(Serial.readStringUntil('\n'));
  }

  // 2. Obsługa komend z WiFi
  WiFiClient newClient = server.available();
  if (newClient) {
    remoteClient = newClient; // Zapamiętaj podłączoną aplikację Python
    Serial.println("Aplikacja Python podłączona przez WiFi");
  }
  
  if (remoteClient && remoteClient.connected() && remoteClient.available()) {
    handleCommand(remoteClient.readStringUntil('\n'));
  }

  // Pętla czasu rzeczywistego (10ms)
  lastMicros = micros();
  unsigned long targetMicros = lastMicros + Ts_micro;
  
  if (runPID) {
    PID_controller();
  } else {
    battery_read();
  }
  
  long time_to_delay = targetMicros - micros();
  if (time_to_delay > 0) delayMicroseconds(time_to_delay);
}

float enc_vel, enc_acc;

void battery_read() {
  M3.setDuty(0);
  float batteryVoltage = battery.getRaw()/236.0;
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  angle = -euler.y();
  
  // Opcjonalne wysyłanie statusu baterii do Seriala
  Serial.print("Battery: "); Serial.print(batteryVoltage, 3);
  Serial.print(", Angle: "); Serial.println(angle, 2);
  delay(1000);
}

void PID_controller() {
  derive(Ts, encoder1.getRawCount(), enc_vel, enc_acc);
  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

  acc = -accel.y() * DEG_TO_RAD;                    
  velocity = gyro.y() * DEG_TO_RAD;       
  angle = -euler.y() * DEG_TO_RAD;

  I *= integral_decay_factor;
  if(abs(angle) > 0.01) I += angle * Ts;
  if (abs(I)>10) I = copysign(10, I);

  float vel_wheel = enc_vel * enc_to_rad;
  float PID = kp * angle + kd * velocity + ki * I + k_omega * vel_wheel;
  
  int dir_PID = 1; 
  if(PID < 0) dir_PID = -1;  
  
  float correction = friction_correction(PID, acc);
  float frict = dir_PID * (friction(enc_vel));
  float fill = PID + frict + correction;
  fill = constrain(fill, -255, 255);

  // --- WYSYŁANIE DANYCH (SERIAL + WIFI) ---
  String dataFrame = String(angle, 4) + "," + String(velocity, 4) + "," + 
                     String(vel_wheel, 4) + "," + String(fill, 2);
  
  // 1. Zawsze wysyłaj po kablu (debugowanie)
  Serial.println(dataFrame);

  // 2. WiFi - tylko jeśli klient jest podłączony (oszczędność energii)
  if (remoteClient && remoteClient.connected()) {
    remoteClient.println(dataFrame);
  }

  // 3. Bluetooth - tylko jeśli ktoś słucha (oszczędność energii)
  if (BLE.connected()) {
    dataChar.writeValue(dataFrame);
  }

  if(abs(angle) < 0.4) M3.setFill(fill);
  else M3.setFill(0);
}