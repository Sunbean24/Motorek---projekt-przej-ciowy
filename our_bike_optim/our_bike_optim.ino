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

volatile float Ts = 0.01;
volatile int Ts_micro = 10000;
volatile float angle = 0.0f, velocity = 0.0f, acc = 0.0f;
volatile int lastMicros = 0, lastCount;

// --- WYNIKI OPTYMALIZACJI BPTT ---
volatile float K_angle = 13.33549213f;   
volatile float K_velocity = -1.19186223f; 
volatile float w_friction = 0.0683f;      
volatile float w_emf = 0.1397f;           

float angle_offset = -1.2f; 

Adafruit_BNO055 bno = Adafruit_BNO055(55);
bool runPID = true;

// --- IDENTYFIKACJA MACIERZY SINDy ---
const float C_matrix[3][8] = {
  //  om_p,     om_w,      u,       sin(th),   tanh(0.5u),  tanh(u),   tanh(2u),   tanh(5u)
  {   1.000f,   0.000f,   0.000f,    0.000f,     0.000f,    0.000f,    0.000f,    0.000f }, 
  {   2.314f,   0.000f,   1.449f, -340.262f,    29.826f,   -8.580f,  -12.456f,  -12.487f }, 
  {   2.155f,  -0.095f,  14.403f,    0.120f,    -6.507f,   -6.899f,   -6.915f,   -6.915f }  
};

BLEService bikeService("19B10000-E8F2-537E-4F6C-D104768A1214");
BLEStringCharacteristic dataChar("19B10001-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify | BLEWrite, 64);

char ssid[] = "Motorek_Projekt"; 
char pass[] = "inzynierka2026";  
WiFiServer server(80);
WiFiClient remoteClient;         

void predict_sindy_dynamics(float th, float om_p, float om_w, float control_u, float* x_dot_out) {
  float features[8];
  features[0] = om_p;
  features[1] = om_w;
  features[2] = control_u;
  features[3] = sin(th);
  features[4] = tanh(0.5f * control_u);
  features[5] = tanh(control_u);
  features[6] = tanh(2.0f * control_u);
  features[7] = tanh(5.0f * control_u);

  for (int i = 0; i < 3; i++) {
    x_dot_out[i] = 0.0f;
    for (int j = 0; j < 8; j++) {
      x_dot_out[i] += C_matrix[i][j] * features[j];
    }
  }
}

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
  if (command.startsWith("setka ")) {
    K_angle = command.substring(6).toFloat();
    Serial.print("K_angle set to: "); Serial.println(K_angle, 6);
  } else if (command.startsWith("setkv ")) {
    K_velocity = command.substring(6).toFloat();
    Serial.print("K_velocity set to: "); Serial.println(K_velocity, 6);
  } else if (command.startsWith("setwf ")) {
    w_friction = command.substring(6).toFloat();
    Serial.print("w_friction set to: "); Serial.println(w_friction, 6);
  } else if (command.startsWith("setkemf ")) {
    w_emf = command.substring(8).toFloat();
    Serial.print("w_emf set to: "); Serial.println(w_emf, 6);
  } else if (command.equals("pid")) {
    runPID = true;
    Serial.println("Running Optimized BPTT controller");
  } else if (command.equals("stop")) {
    runPID = false;
    M2.setDuty(0);
    M3.setDuty(0);
    Serial.println("Stopping / Battery mode");
  }
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(5);
  
  unsigned long startWait = millis();
  while (!Serial && millis() - startWait < 3000) { ; }
  
  setup_wifi(); 
  setup_ble();

  if (!bno.begin()) {
    Serial.println("Failed to initialize BNO055!");
    while (1);
  }
  
  delay(1000);
  bno.setExtCrystalUse(true);
  
  //bno.setMode(adafruit_bno055_opmode_t::OPERATION_MODE_IMUPLUS);
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
  // if (dataChar.written()) {
  //   String bleCommand = dataChar.value();
  //   Serial.print("BLE Command: "); Serial.println(bleCommand);
  //   handleCommand(bleCommand);
  // }

  if (Serial.available()) {
    handleCommand(Serial.readStringUntil('\n'));
  }

  // WiFiClient newClient = server.available();
  // if (newClient) {
  //   remoteClient = newClient; 
  //   remoteClient.setTimeout(5);
  //   Serial.println("Aplikacja Python podłączona przez WiFi");
  // }
  
  // if (remoteClient && remoteClient.connected() && remoteClient.available()) {
  //   handleCommand(remoteClient.readStringUntil('\n'));
  // }

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
  //delay(1000);
}

void PID_controller() {
  long enkoder = encoder1.getRawCount();
  derive(Ts, enkoder, enc_vel, enc_acc);
  
  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

  acc = -accel.y();                    
  velocity = gyro.y();       
  
  float corrected_angle_deg = -euler.y() - angle_offset;
  angle = corrected_angle_deg * DEG_TO_RAD;

  float vel_wheel = enc_vel * enc_to_rad;

  float u_raw = -(K_angle * angle + K_velocity * velocity);
  float u = u_raw + w_friction * tanh(5.0f * u_raw) + w_emf * vel_wheel;
  u = constrain(u, -12.0f, 12.0f);

  float fill = (u / 12.0f) * 255.0f;

  float x_dot[3];
  predict_sindy_dynamics(angle, velocity, vel_wheel, u, x_dot);

  float timestamp_s = millis() / 1000.0f;

  String dataFrame = String(timestamp_s, 3) + "," + 
                   String(angle, 4) + "," + 
                   String(velocity, 4) + "," + 
                   String(acc, 4) + "," + 
                   String(enkoder * enc_to_rad, 4) + "," + 
                   String(fill, 2);
  
  Serial.println(dataFrame);

  if (remoteClient && remoteClient.connected()) {
    remoteClient.println(dataFrame);
  }

  if (BLE.connected()) {
    dataChar.writeValue(dataFrame);
  }

  if (abs(angle) < (10.0f * DEG_TO_RAD)) M3.setFill(fill);
  else M3.setFill(0);
}
