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
#define DEG_TO_RAD 0.01745329

volatile float I = 0, Ts = 0.01;
volatile int Ts_micro = 10000;
volatile float kp = 4000.0f, ki = 30.0f, kd = 500.0f, k_omega = -0.007;
volatile float integral_decay_factor = 0.9999f;
volatile float angle = 0.0f, velocity = 0.0f, acc = 0.0f;
volatile int lastMicros = 0, lastCount;

float angle_offset = -1.2f; 

Adafruit_BNO055 bno = Adafruit_BNO055(55);
bool runPID = true;

BLEService bikeService("19B10000-E8F2-537E-4F6C-D104768A1214");
BLEStringCharacteristic dataChar("19B10001-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify | BLEWrite, 64);

char ssid[] = "Motorek_Projekt"; 
char pass[] = "inzynierka2026";  
WiFiServer server(80);
WiFiClient remoteClient;         

void setup_ble() {
  if (!BLE.begin()) return;
  BLE.setLocalName("Bike-Project");
  BLE.setAdvertisedService(bikeService);
  bikeService.addCharacteristic(dataChar);
  BLE.addService(bikeService);
  BLE.advertise();
}

void setup_wifi() {
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Błąd: Brak modułu WiFi!");
    return;
  }
  
  Serial.print("Tworzenie sieci AP: ");
  Serial.println(ssid);
  
  if (WiFi.beginAP(ssid, pass) != WL_AP_LISTENING) {
    Serial.println("Błąd tworzenia punktu dostępowego!");
    while (1);
  }
  
  server.begin();
  Serial.print("IP motorka: "); 
  Serial.println(WiFi.localIP()); 
}

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
  
  setup_wifi(); 
  setup_ble();

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
  if (dataChar.written()) {
    String bleCommand = dataChar.value();
    Serial.print("BLE Command: "); Serial.println(bleCommand);
    handleCommand(bleCommand);
  }

  if (Serial.available()) {
    handleCommand(Serial.readStringUntil('\n'));
  }

  WiFiClient newClient = server.available();
  if (newClient) {
    remoteClient = newClient; 
    Serial.println("Aplikacja Python podłączona przez WiFi");
  }
  
  if (remoteClient && remoteClient.connected() && remoteClient.available()) {
    handleCommand(remoteClient.readStringUntil('\n'));
  }

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
  float batteryVoltage = battery.getRaw() / 236.0;
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  
  angle = -euler.y() - angle_offset;
  
  Serial.print("Battery: "); Serial.print(batteryVoltage, 3);
  Serial.print(", Angle: "); Serial.println(angle, 2);
}

void PID_controller() {
  long raw_pulses = encoder1.getRawCount();
  derive(Ts, raw_pulses, enc_vel, enc_acc);
  
  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

  acc = -accel.y();               
  velocity = gyro.y();            
  
  float corrected_angle_deg = -euler.y() - angle_offset;
  angle = corrected_angle_deg * DEG_TO_RAD; 

  I *= integral_decay_factor;
  if(abs(angle) > 0.01) I += angle * Ts;
  if (abs(I)>10) I = copysign(10, I);

  float vel_wheel = enc_vel * enc_to_rad; 
  float PID = kp * angle + kd * velocity + ki * I + k_omega * vel_wheel;
  
  int dir_PID = (PID < 0) ? -1 : 1; 
  
  float correction = friction_correction(PID, acc);
  float frict = dir_PID * (friction(enc_vel));
  float fill = constrain(PID + frict + correction, -255, 255);

  float timestamp_s = millis() / 1000.0; 

  // --- ZMIANA: Przeliczenie impulsów na przebyty kąt koła w radianach ---
  float wheel_angle_rad = raw_pulses * enc_to_rad;

  String dataFrame = String(timestamp_s, 3) + "," + 
                   String(angle, 4) + "," + 
                   String(velocity, 4) + "," + 
                   String(acc, 4) + "," + 
                   String(wheel_angle_rad, 4) + "," + 
                   String(fill, 2);
  
  Serial.println(dataFrame);

  if (remoteClient && remoteClient.connected()) {
    remoteClient.println(dataFrame);
  }

  if (BLE.connected()) {
    dataChar.writeValue(dataFrame);
  }

  if(abs(angle) < (10.0f * DEG_TO_RAD)) {
    M3.setFill(fill);
  } else {
    M3.setFill(0);
  }
}