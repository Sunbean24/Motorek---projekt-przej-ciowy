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

// --- PARAMETRY FIZYCZNE EKF ---
#define GRAVITY_EKF 9.81f
#define PENDULUM_LENGTH 0.15f  // Odległość środka masy wahadła do osi
bool useEKF = true;
volatile float I = 0, Ts = 0.01;
volatile int Ts_micro = 10000;
volatile int lastMicros = 0, lastCount;

volatile float integral_decay_factor = 0.9999f; 
float angle_offset = -1.2f; 

Adafruit_BNO055 bno = Adafruit_BNO055(55);
bool runPID = true;

// --- ZMIENNE EKF ---
volatile float x_hat[3] = {0.0f, 0.0f, 0.0f};
volatile float P_cov[3][3] = {
  {0.1f, 0.0f, 0.0f},
  {0.0f, 0.1f, 0.0f},
  {0.0f, 0.0f, 0.1f}
};
volatile float last_u = 0.0f;
// -------------------

// Indeksy: 0 = K_angle, 1 = K_velocity, 2 = K_wheel, 3 = K_integral
volatile float K_gains[4] = {-149.97158813f, -18.62984467f, 0.06907678f, -9.17101669f};
volatile float w_friction = -7.5654f;
volatile float b_friction = -3837.3443f;
volatile float w_emf = -0.0025f;
volatile float w_aero = 0.00000000f;

const float C_matrix[3][8] = {
  { 1.0000f, 0.0000f, 0.0000f, 0.0000f, 0.0000f, 0.0000f, 0.0000f, 0.0000f },
  { -0.5716f, 0.0000f, 1.0828f, -484.4104f, -5.3070f, 1.5706f, 1.6194f, 1.6168f },
  { -4.8924f, 0.0000f, 0.2094f, 1692.1262f, 521.4197f, -121.9280f, -182.4950f, -182.9740f }
};

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
  if (WiFi.status() == WL_NO_MODULE) return;
  WiFi.beginAP(ssid, pass);
  server.begin();
}

void handleCommand(String command) {
  command.trim();
  if (command.startsWith("setka ")) K_gains[0] = command.substring(6).toFloat();
  else if (command.startsWith("setkv ")) K_gains[1] = command.substring(6).toFloat();
  else if (command.startsWith("setkw ")) K_gains[2] = command.substring(6).toFloat();
  else if (command.startsWith("setki ")) K_gains[3] = command.substring(6).toFloat();
  else if (command.startsWith("setoffs ")) angle_offset = command.substring(6).toFloat();
  else if (command.equals("pid")) { runPID = true; I = 0; }
  else if (command.equals("stop")) { runPID = false; M3.setDuty(0); }
  else if (command.equals("ekf on")) { useEKF = true; Serial.println("EKF ON"); }
  else if (command.equals("ekf off")) { useEKF = false; Serial.println("EKF OFF"); }
}

void predict_sindy_dynamics(float th, float om_p, float om_w, float control_u, float* x_dot_out) {
  float features[8] = {
    om_p, om_w, control_u, sin(th),
    tanh(0.5f * control_u), tanh(control_u),
    tanh(2.0f * control_u), tanh(5.0f * control_u)
  };
  for (int i = 0; i < 3; i++) {
    x_dot_out[i] = 0.0f;
    for (int j = 0; j < 8; j++) x_dot_out[i] += C_matrix[i][j] * features[j];
  }
}

void ekf_update_1d(float H[3], float z_meas, float z_pred, float R_val, float x[3], float P[3][3]) {
  float PHt[3] = {0, 0, 0};
  for(int i=0; i<3; i++)
    for(int j=0; j<3; j++)
      PHt[i] += P[i][j] * H[j];
      
  float S = R_val;
  for(int i=0; i<3; i++) S += H[i] * PHt[i];
  
  float K[3];
  for(int i=0; i<3; i++) K[i] = PHt[i] / S;
  
  float inn = z_meas - z_pred;
  for(int i=0; i<3; i++) x[i] += K[i] * inn;
  
  float HP[3] = {0, 0, 0};
  for(int j=0; j<3; j++)
    for(int i=0; i<3; i++)
      HP[j] += H[i] * P[i][j];
      
  for(int i=0; i<3; i++)
    for(int j=0; j<3; j++)
      P[i][j] -= K[i] * HP[j];
}

// ZWRÓCONY AKCELEROMETR (z_acc)
void run_ekf(float z_angle, float z_gyro, float z_acc, float z_wheel) {
  const float dt = Ts; 
  const float eps = 1e-4f;
  
  // 1. Krok predykcji stanu
  float x_dot[3];
  predict_sindy_dynamics(x_hat[0], x_hat[1], x_hat[2], last_u, x_dot);
  
  float x_pred[3];
  for(int i=0; i<3; i++) x_pred[i] = x_hat[i] + x_dot[i] * dt;
  
  float F[3][3];
  for(int i=0; i<3; i++) {
    float x_plus[3] = {x_hat[0], x_hat[1], x_hat[2]};
    x_plus[i] += eps;
    float dot_plus[3];
    predict_sindy_dynamics(x_plus[0], x_plus[1], x_plus[2], last_u, dot_plus);
    for(int j=0; j<3; j++) {
      F[j][i] = (i == j ? 1.0f : 0.0f) + ((dot_plus[j] - x_dot[j]) / eps) * dt;
    }
  }
  
  float FP[3][3] = {0};
  for(int i=0; i<3; i++)
    for(int j=0; j<3; j++)
      for(int k=0; k<3; k++)
        FP[i][j] += F[i][k] * P_cov[k][j];
        
  float P_pred[3][3] = {0};
  float Q[3] = {1e-4f, 1e-2f, 1e-3f}; 
  for(int i=0; i<3; i++) {
    for(int j=0; j<3; j++) {
      P_pred[i][j] = (i == j ? Q[i] : 0.0f);
      for(int k=0; k<3; k++) {
        P_pred[i][j] += FP[i][k] * F[j][k];
      }
    }
  }
  
  // 2. Krok aktualizacji
  float H0[3] = {1.0f, 0.0f, 0.0f};
  ekf_update_1d(H0, z_angle, x_pred[0], 1e-2f, x_pred, P_pred);
  
  float H1[3] = {0.0f, 1.0f, 0.0f};
  ekf_update_1d(H1, z_gyro, x_pred[1], 1e-2f, x_pred, P_pred);
  
  float H3[3] = {0.0f, 0.0f, 1.0f};
  ekf_update_1d(H3, z_wheel, x_pred[2], 2.0f, x_pred, P_pred);

  // Aktualizacja dla akcelerometru
  float x_dot_pred[3];
  predict_sindy_dynamics(x_pred[0], x_pred[1], x_pred[2], last_u, x_dot_pred);
  float z_pred2 = GRAVITY_EKF * sin(x_pred[0]) + PENDULUM_LENGTH * x_dot_pred[1];
  
  float H2[3];
  for(int i=0; i<3; i++) {
    float xp[3] = {x_pred[0], x_pred[1], x_pred[2]};
    xp[i] += eps;
    float xdp[3];
    predict_sindy_dynamics(xp[0], xp[1], xp[2], last_u, xdp);
    float zp = GRAVITY_EKF * sin(xp[0]) + PENDULUM_LENGTH * xdp[1];
    H2[i] = (zp - z_pred2) / eps;
  }
  // R powiększone (0.5f), żeby akcelerometr korygował delikatnie, a nie szarpał
  ekf_update_1d(H2, z_acc, z_pred2, 0.5f, x_pred, P_pred); 
  
  // Zapis do zmiennych globalnych
  for(int i=0; i<3; i++) {
    x_hat[i] = x_pred[i];
    for(int j=0; j<3; j++) P_cov[i][j] = P_pred[i][j];
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
  if (Serial.available()) handleCommand(Serial.readStringUntil('\n'));

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
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  
  float corrected_angle_deg = -euler.y() - angle_offset;
  float current_angle = corrected_angle_deg * DEG_TO_RAD;

  // W trybie spoczynku twardo resetujemy EKF
  x_hat[0] = current_angle;
  x_hat[1] = gyro.y();
  x_hat[2] = 0.0f;
  
  for(int i=0; i<3; i++)
    for(int j=0; j<3; j++)
      P_cov[i][j] = (i==j) ? 0.1f : 0.0f;

  Serial.print("Battery: "); Serial.print(batteryVoltage, 3);
  Serial.print(", Angle: "); Serial.println(current_angle, 4);
}

void PID_controller() {
  long long enkoder = encoder1.getRawCount();
  derive(Ts, enkoder, enc_vel, enc_acc);
  
  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

  float z_acc = -accel.y(); 
  float z_gyro = gyro.y();       
  float corrected_angle_deg = -euler.y() - angle_offset;
  float z_angle = corrected_angle_deg * DEG_TO_RAD;
  float z_wheel = enc_vel * enc_to_rad;

  float est_angle, est_vel, est_vel_wheel;

  if (useEKF) {
    // --- MECHANIZM RESETU CYKLICZNEGO / ZABEZPIECZENIE PRZED DRYFEM ---
    static unsigned long last_reset_time = 0;
    bool angle_diverged = abs(x_hat[0] - z_angle) > 0.05f; 
    bool periodic_reset = (millis() - last_reset_time > 100); 
    
    if (angle_diverged || periodic_reset) {
      // Resetujemy kowariancję do wartości początkowych, by pobudzić EKF do patrzenia na pomiary
      for(int i=0; i<3; i++) {
        for(int j=0; j<3; j++) {
          P_cov[i][j] = (i==j) ? 0.1f : 0.0f;
        }
      }
      // Jeśli różnica była niebezpiecznie duża, "ściągamy" stan do wskazań czujnika
      if (angle_diverged) {
        x_hat[0] = z_angle;
      }
      last_reset_time = millis();
    }
    // ------------------------------------------------------------------

    // Fuzja Z akcelerometrem
    run_ekf(z_angle, z_gyro, z_acc, z_wheel);
    
    est_angle = x_hat[0];
    est_vel = x_hat[1];
    est_vel_wheel = x_hat[2];
  } else {
    // --- BYPASS EKF: Użycie surowych danych z czujników ---
    est_angle = z_angle;
    est_vel = z_gyro;
    est_vel_wheel = z_wheel;

    // Aktualizujemy stan EKF w tle, aby po jego włączeniu nie było nagłego szarpnięcia
    x_hat[0] = z_angle;
    x_hat[1] = z_gyro;
    x_hat[2] = z_wheel;
    for(int i=0; i<3; i++) {
      for(int j=0; j<3; j++) {
        P_cov[i][j] = (i==j) ? 0.1f : 0.0f;
      }
    }
  }

  I *= integral_decay_factor;
  if(abs(est_angle) > 0.01) I += est_angle * Ts;
  if (abs(I)>10.0f) I = copysign(10.0f, I);
  
  float u_raw = -(K_gains[0] * est_angle + K_gains[1] * est_vel + K_gains[2] * est_vel_wheel + K_gains[3] * I);
  float u_ff = w_friction * tanh(b_friction * est_vel_wheel) + w_emf * est_vel_wheel + w_aero * est_vel_wheel * est_vel_wheel * est_vel_wheel;
  
  float u = u_raw + u_ff;
  u = constrain(u, -12.0f, 12.0f);
  
  last_u = u; 

  float fill = (u / 12.0f) * 255.0f;
  fill = constrain(fill, -255.0f, 255.0f);

  unsigned long timestamp = millis();

  String dataFrame = String(float(timestamp)/1000.0f, 3) + "," + 
                     String(est_angle, 4) + "," + 
                     String(est_vel, 4) + "," +
                     String(z_acc, 4) + "," + 
                     String(est_vel_wheel, 4) + "," + 
                     String(fill, 2);
  
  Serial.println(dataFrame);
  if (remoteClient && remoteClient.connected()) remoteClient.println(dataFrame);
  if (BLE.connected()) dataChar.writeValue(dataFrame);

  if(abs(z_angle) < (10.0f * DEG_TO_RAD)) M3.setFill(fill);
  else M3.setFill(0);
}