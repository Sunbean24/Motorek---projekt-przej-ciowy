#include <ArduinoBLE.h>
#include "DCMotor.h"
#include <ArduinoMotorCarrier.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Arduino.h>
#include <WiFiNINA.h>
#include "friction.h" 
#include "filter.h"

#define enc_to_rad 0.1308996938995f
#define DEG_TO_RAD 0.01745329f // pi/180

#define GRAVITY_EKF 9.81f
#define PENDULUM_LENGTH 0.15f
bool useEKF = true;
volatile float I = 0, Ts = 0.01;
volatile int Ts_micro = 10000;
volatile int lastMicros = 0, lastCount;

volatile float integral_decay_factor = 0.9999f; 
float angle_offset = -1.2f; 

#define TELEMETRY_EVERY 1   // co ile cykli wysyłać telemetrię (1 = każdy krok)

// --- TABLICE STAŁOPRZECINKOWE Q15 DLA FUNKCJI PRZESTĘPNYCH ---
#define SIN_N 256
#define TANH_N 256
#define TANH_MAX 4.0f
static int16_t SIN_LUT[SIN_N + 1];
static int16_t TANH_LUT[TANH_N + 1];
float SIN_SCALE;
const float TANH_SCALE = (float)TANH_N / TANH_MAX;

// --- SPARSE C (indeksy niezerowych kolumn per wiersz) ---
int8_t C_nz[3][8];
int8_t C_nz_count[3];

Adafruit_BNO055 bno = Adafruit_BNO055(55);
bool runPID = true;

volatile float x_hat[3] = {0.0f, 0.0f, 0.0f};
volatile float P_cov[3][3] = {
  {0.1f, 0.0f, 0.0f},
  {0.0f, 0.1f, 0.0f},
  {0.0f, 0.0f, 0.1f}
};
volatile float last_u = 0.0f;

const float PANIC_THRESHOLD = 0.02f;
const float RECOVERY_LOOK_AHEAD = 0.15f;

// --- LQR: jeden krok Riccatiego na próbkę, P persystentne między próbkami ---
#define LQR_MAX_ITER 200      // tylko do jednorazowej konwergencji nominału
#define LQR_CONV_TOL 0.5f     // tolerancja zbieżności nominału (dominuje K_angle)
bool useLQR = true;
bool lqr_ready = false;
const float GAIN_MAX_FACTOR = 5.0f;
volatile float Q_lqr[3] = {2000.0f, 8.0f, 0.02f};
volatile float R_lqr = 1.0f;
float K_lqr_nominal[3] = {0.0f, 0.0f, 0.0f};
float K_lqr[3] = {0.0f, 0.0f, 0.0f};
float P_lqr[3][3];            // stan value iteration, niesiony między próbkami

int signum(float num)
{
  if (num > 0) return 1;
  if (num == 0) return 0;
  return -1;
}
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

void build_luts() {
  SIN_SCALE = (float)SIN_N / HALF_PI;
  for (int i = 0; i <= SIN_N; i++) {
    float x = HALF_PI * (float)i / (float)SIN_N;
    SIN_LUT[i] = (int16_t)(sinf(x) * 32767.0f + 0.5f);
  }
  for (int i = 0; i <= TANH_N; i++) {
    float x = TANH_MAX * (float)i / (float)TANH_N;
    TANH_LUT[i] = (int16_t)(tanhf(x) * 32767.0f + 0.5f);
  }
}

void build_sparse() {
  for (int i = 0; i < 3; i++) {
    int c = 0;
    for (int j = 0; j < 8; j++)
      if (C_matrix[i][j] != 0.0f) C_nz[i][c++] = (int8_t)j;
    C_nz_count[i] = (int8_t)c;
  }
}

float tanh_fast(float x) {
  float sign = 1.0f;
  if (x < 0.0f) { x = -x; sign = -1.0f; }
  if (x >= TANH_MAX) return sign;
  float fidx = x * TANH_SCALE;
  int i = (int)fidx;
  float frac = fidx - (float)i;
  int a = TANH_LUT[i];
  int b = TANH_LUT[i + 1];
  return sign * ((a + (b - a) * frac) * (1.0f / 32767.0f));
}

float sin_fast(float x) {
  if (!isfinite(x)) return 0.0f;
  while (x > PI) x -= TWO_PI;
  while (x < -PI) x += TWO_PI;
  float sign = 1.0f;
  if (x < 0.0f) { x = -x; sign = -1.0f; }
  if (x > HALF_PI) x = PI - x;
  float fidx = x * SIN_SCALE;
  int i = (int)fidx;
  if (i >= SIN_N) i = SIN_N - 1;
  float frac = fidx - (float)i;
  int a = SIN_LUT[i];
  int b = SIN_LUT[i + 1];
  return sign * ((a + (b - a) * frac) * (1.0f / 32767.0f));
}

float cos_fast(float x) { return sin_fast(x + HALF_PI); }

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

void sindy_eval(float th, float om_p, float om_w, float u,
                float x_dot[3], float A[3][3], float B[3]) {
  float s = sin_fast(th), c = cos_fast(th);
  float t05 = tanh_fast(0.5f * u), t1 = tanh_fast(u), t2 = tanh_fast(2.0f * u), t5 = tanh_fast(5.0f * u);
  float f[8]  = { om_p, om_w, u, s, t05, t1, t2, t5 };
  float fu[8] = { 0.0f, 0.0f, 1.0f, 0.0f,
                  0.5f * (1.0f - t05 * t05),
                  (1.0f - t1 * t1),
                  2.0f * (1.0f - t2 * t2),
                  5.0f * (1.0f - t5 * t5) };
  for (int i = 0; i < 3; i++) {
    float xd = 0.0f, b = 0.0f;
    int cnt = C_nz_count[i];
    for (int n = 0; n < cnt; n++) {
      int j = C_nz[i][n];
      float cij = C_matrix[i][j];
      xd += cij * f[j];
      b  += cij * fu[j];
    }
    x_dot[i] = xd;
    B[i] = b;
    A[i][0] = C_matrix[i][3] * c;
    A[i][1] = C_matrix[i][0];
    A[i][2] = C_matrix[i][1];
  }
}

void operating_point_AdBd(float th, float u, float Ad[3][3], float Bd[3]) {
  float x_dot[3], A[3][3], B[3];
  sindy_eval(th, 0.0f, 0.0f, u, x_dot, A, B);
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) Ad[i][j] = (i == j ? 1.0f : 0.0f) + A[i][j] * Ts;
    Bd[i] = B[i] * Ts;
  }
}

static inline float clamp_gain_band(float k, float k0, float factor) {
  if (k0 == 0.0f) return 0.0f;
  float a = k0 / factor;
  float b = k0 * factor;
  float lo = (a < b) ? a : b;
  float hi = (a < b) ? b : a;
  return (k < lo) ? lo : ((k > hi) ? hi : k);
}

// Jeden krok wstecznej rekursji Riccatiego. Aktualizuje P w miejscu, zwraca K.
bool lqr_step(const float Ad[3][3], const float Bd[3], float P[3][3], float K_out[3]) {
  float PB[3] = {0, 0, 0};
  for (int i = 0; i < 3; i++)
    for (int k = 0; k < 3; k++) PB[i] += P[i][k] * Bd[k];

  float R_eff = R_lqr;
  for (int i = 0; i < 3; i++) R_eff += Bd[i] * PB[i];
  if (!(R_eff > 1e-9f)) return false;

  float bP[3] = {0, 0, 0};
  for (int j = 0; j < 3; j++)
    for (int i = 0; i < 3; i++) bP[j] += Bd[i] * P[i][j];

  float bPA[3] = {0, 0, 0};
  for (int j = 0; j < 3; j++)
    for (int k = 0; k < 3; k++) bPA[j] += bP[k] * Ad[k][j];

  for (int j = 0; j < 3; j++) K_out[j] = bPA[j] / R_eff;

  float Acl[3][3];
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++) Acl[i][j] = Ad[i][j] - Bd[i] * K_out[j];

  float PAcl[3][3] = {0};
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
      for (int k = 0; k < 3; k++) PAcl[i][j] += P[i][k] * Acl[k][j];

  float Pn[3][3] = {0};
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++) {
      float v = (i == j ? Q_lqr[i] : 0.0f) + R_lqr * K_out[i] * K_out[j];
      for (int k = 0; k < 3; k++) v += Acl[k][i] * PAcl[k][j];
      Pn[i][j] = v;
    }

  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++) P[i][j] = Pn[i][j];

  for (int j = 0; j < 3; j++) if (!isfinite(K_out[j])) return false;
  return true;
}

// Pełna konwergencja (tylko nominał w setup/setq*). P seedowane = Q.
bool lqr_converge(const float Ad[3][3], const float Bd[3], float P[3][3], float K_out[3]) {
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++) P[i][j] = (i == j ? Q_lqr[i] : 0.0f);

  float Kp[3] = {0.0f, 0.0f, 0.0f};
  for (int s = 0; s < LQR_MAX_ITER; s++) {
    if (!lqr_step(Ad, Bd, P, K_out)) return false;
    float dmax = 0.0f;
    for (int j = 0; j < 3; j++) {
      float d = fabsf(K_out[j] - Kp[j]);
      if (d > dmax) dmax = d;
    }
    if (s > 0 && dmax < LQR_CONV_TOL) return true;
    for (int j = 0; j < 3; j++) Kp[j] = K_out[j];
  }
  return true;
}

void rebuild_lqr() {
  float Ad[3][3], Bd[3];
  operating_point_AdBd(0.0f, 0.0f, Ad, Bd);
  lqr_ready = lqr_converge(Ad, Bd, P_lqr, K_lqr_nominal);  // P_lqr zostaje zbieżne = seed runtime
}

void handleCommand(String command) {
  command.trim();
  if (command.startsWith("setka ")) K_gains[0] = command.substring(6).toFloat();
  else if (command.startsWith("setkv ")) K_gains[1] = command.substring(6).toFloat();
  else if (command.startsWith("setkw ")) K_gains[2] = command.substring(6).toFloat();
  else if (command.startsWith("setki ")) K_gains[3] = command.substring(6).toFloat();
  else if (command.startsWith("setoffs ")) angle_offset = command.substring(6).toFloat();
  else if (command.startsWith("setq0 ")) { Q_lqr[0] = command.substring(6).toFloat(); rebuild_lqr(); }
  else if (command.startsWith("setq1 ")) { Q_lqr[1] = command.substring(6).toFloat(); rebuild_lqr(); }
  else if (command.startsWith("setq2 ")) { Q_lqr[2] = command.substring(6).toFloat(); rebuild_lqr(); }
  else if (command.startsWith("setr ")) { R_lqr = command.substring(5).toFloat(); rebuild_lqr(); }
  else if (command.equals("pid")) { runPID = true; I = 0; }
  else if (command.equals("stop")) { runPID = false; M3.setDuty(0); }
  else if (command.equals("ekf on")) { useEKF = true; Serial.println("EKF ON"); }
  else if (command.equals("ekf off")) { useEKF = false; Serial.println("EKF OFF"); }
  else if (command.equals("lqr on")) { useLQR = true; rebuild_lqr(); Serial.println("LQR ON"); }
  else if (command.equals("lqr off")) { useLQR = false; Serial.println("LQR OFF"); }
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

void run_ekf(float z_angle, float z_gyro, float z_acc, float z_wheel) {
  const float dt = Ts;

  float x_dot[3], A[3][3], B[3];
  sindy_eval(x_hat[0], x_hat[1], x_hat[2], last_u, x_dot, A, B);

  float x_pred[3];
  for (int i = 0; i < 3; i++) x_pred[i] = x_hat[i] + x_dot[i] * dt;

  float F[3][3];
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
      F[i][j] = (i == j ? 1.0f : 0.0f) + A[i][j] * dt;

  float FP[3][3] = {0};
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
      for (int k = 0; k < 3; k++)
        FP[i][j] += F[i][k] * P_cov[k][j];

  float P_pred[3][3] = {0};
  float Q[3] = {1e-4f, 1e-2f, 1e-3f};
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++) {
      P_pred[i][j] = (i == j ? Q[i] : 0.0f);
      for (int k = 0; k < 3; k++) P_pred[i][j] += FP[i][k] * F[j][k];
    }

  float H0[3] = {1.0f, 0.0f, 0.0f};
  ekf_update_1d(H0, z_angle, x_pred[0], 1e-2f, x_pred, P_pred);
  float H1[3] = {0.0f, 1.0f, 0.0f};
  ekf_update_1d(H1, z_gyro, x_pred[1], 1e-2f, x_pred, P_pred);
  float H3[3] = {0.0f, 0.0f, 1.0f};
  ekf_update_1d(H3, z_wheel, x_pred[2], 2.0f, x_pred, P_pred);

  float x_dot_pred[3], A2[3][3], B2[3];
  sindy_eval(x_pred[0], x_pred[1], x_pred[2], last_u, x_dot_pred, A2, B2);
  float z_pred2 = GRAVITY_EKF * sin_fast(x_pred[0]) + PENDULUM_LENGTH * x_dot_pred[1];
  float H2[3];
  H2[0] = GRAVITY_EKF * cos_fast(x_pred[0]) + PENDULUM_LENGTH * A2[1][0];
  H2[1] = PENDULUM_LENGTH * A2[1][1];
  H2[2] = PENDULUM_LENGTH * A2[1][2];
  ekf_update_1d(H2, z_acc, z_pred2, 0.5f, x_pred, P_pred);

  for (int i = 0; i < 3; i++) {
    x_hat[i] = x_pred[i];
    for (int j = 0; j < 3; j++) P_cov[i][j] = P_pred[i][j];
  }
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(5); 
  
  unsigned long startWait = millis();
  while (!Serial && millis() - startWait < 3000) { ; }

  build_luts();
  build_sparse();

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

  rebuild_lqr();
  for (int i = 0; i < 3; i++) K_lqr[i] = K_gains[i];
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
  float batteryVoltage = battery.getRaw()/236.0f;
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  
  float corrected_angle_deg = -euler.y() - angle_offset;
  float current_angle = corrected_angle_deg * DEG_TO_RAD;

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
    static unsigned long last_reset_time = 0;
    bool angle_diverged = abs(x_hat[0] - z_angle) > 0.05f; 
    bool periodic_reset = (millis() - last_reset_time > 100); 
    
    if (angle_diverged || periodic_reset) {
      for(int i=0; i<3; i++) {
        for(int j=0; j<3; j++) {
          P_cov[i][j] = (i==j) ? 0.1f : 0.0f;
        }
      }
      if (angle_diverged) {
        x_hat[0] = z_angle;
      }
      last_reset_time = millis();
    }

    run_ekf(z_angle, z_gyro, z_acc, z_wheel);
    
    est_angle = x_hat[0];
    est_vel = x_hat[1];
    est_vel_wheel = x_hat[2];
  } else {
    est_angle = z_angle;
    est_vel = z_gyro;
    est_vel_wheel = z_wheel;

    x_hat[0] = z_angle;
    x_hat[1] = z_gyro;
    x_hat[2] = z_wheel;
    for(int i=0; i<3; i++) {
      for(int j=0; j<3; j++) {
        P_cov[i][j] = (i==j) ? 0.1f : 0.0f;
      }
    }
  }

  if (useLQR && lqr_ready) {
    float Ad[3][3], Bd[3];
    operating_point_AdBd(est_angle, last_u, Ad, Bd);
    float Kc[3];
    if (lqr_step(Ad, Bd, P_lqr, Kc)) {
      for (int i = 0; i < 3; i++) {
        float scheduled = K_gains[i] + (Kc[i] - K_lqr_nominal[i]);
        K_lqr[i] = clamp_gain_band(scheduled, K_gains[i], GAIN_MAX_FACTOR);
      }
    } else {
      for (int i = 0; i < 3; i++)             // reseed po niepowodzeniu (R_eff<=0 / NaN)
        for (int j = 0; j < 3; j++) P_lqr[i][j] = (i == j ? Q_lqr[i] : 0.0f);
    }
  } else {
    for (int i = 0; i < 3; i++) K_lqr[i] = K_gains[i];
  }

  I *= integral_decay_factor;
  if(abs(est_angle) > 0.01) I += est_angle * Ts;
  if (abs(I)>10.0f) I = copysign(10.0f, I);
  
  float u_raw = -(K_lqr[0] * est_angle + K_lqr[1] * est_vel + K_lqr[2] * est_vel_wheel + K_gains[3] * I);
  float u_ff = w_friction * tanh_fast(b_friction * est_vel_wheel) + w_emf * est_vel_wheel + w_aero * est_vel_wheel * est_vel_wheel * est_vel_wheel;
  
  float u = u_raw + u_ff;
  u = constrain(u, -12.0f, 12.0f);
  if (est_angle * est_vel > 0.0f) {
    float predicted_angle = est_angle + est_vel * RECOVERY_LOOK_AHEAD;
    if (abs(predicted_angle) > PANIC_THRESHOLD) {
      u = 12.0f * signum(est_angle);
    }
  }
  last_u = u; 

  float fill = (u / 12.0f) * 255.0f;
  fill = constrain(fill, -255.0f, 255.0f);

  static unsigned int tel_div = 0;
  if (tel_div == 0) {
    char frame[80];
    snprintf(frame, sizeof(frame), "%.3f,%.4f,%.4f,%.4f,%.4f,%.2f",
             millis() / 1000.0f, est_angle, est_vel, z_acc, est_vel_wheel, fill);

    Serial.println(frame);
    if (remoteClient && remoteClient.connected()) remoteClient.println(frame);
    if (BLE.connected()) dataChar.writeValue(frame);
  }
  tel_div = (tel_div + 1) % TELEMETRY_EVERY;

  if(abs(z_angle) < (10.0f * DEG_TO_RAD)) M3.setFill(fill);
  else M3.setFill(0);
}