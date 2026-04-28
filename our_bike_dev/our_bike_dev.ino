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
// Variable to control which function runs in the loop
bool runPID = true;

char ssid[] = "TWOJA_NAZWA_WIFI";
char pass[] = "TWOJE_HASLO";
WiFiServer server(80);

void setup_wifi() {
  if (WiFi.status() == WL_NO_MODULE) return;
  while (WiFi.begin(ssid, pass) != WL_CONNECTED) delay(500);
  server.begin();
  Serial.print("IP: "); Serial.println(WiFi.localIP());
}

void setup() {
  Serial.begin(115200);
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
  M1.setDuty(0);
  M2.setDuty(0);
  M3.setDuty(0);
  M4.setDuty(0);
  servo2.setAngle(100);
  delay(100);
  servo2.detach();
  // Serial.print(friction(3000)); Serial.print(" "); Serial.println(friction(-3000));
}

void loop() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    if (command.startsWith("setkp ")) {
      kp = command.substring(6).toFloat();
      Serial.print("kp set to: ");
      Serial.println(kp,6);
    } else if (command.startsWith("setki ")) {
      ki = command.substring(6).toFloat();
      Serial.print("ki set to: ");
      Serial.println(ki,6);
    } else if (command.startsWith("setkd ")) {
      kd = command.substring(6).toFloat();
      Serial.print("kd set to: ");
      Serial.println(kd,6);
    } else if (command.startsWith("setkomega ")) {
      k_omega = command.substring(10).toFloat();
      Serial.print("k_omega set to: ");
      Serial.println(k_omega, 6);
    } else if (command.equals("pid")) {
      runPID = true;
      I = 0;
      friction_correction_integral = 0;
      Serial.println("Running PID controller");
    } 
     else if (command.equals("run")) {
      M2.setDuty(55);
      Serial.println("Running");
    }else if (command.equals("stop")) {
      runPID = false;
      M2.setDuty(0);
      Serial.println("Running Battery charging");
    } else {
      Serial.println("Invalid command. Use 'setkp <value>', 'setki <value>', 'setkd <value>','setomega <value>' 'pid', or 'stop'.");
    }
  }

  lastMicros = micros();
  unsigned long targetMicros = lastMicros + Ts_micro;
  long time_to_delay = targetMicros - micros();
  delayMicroseconds(time_to_delay);

  if (runPID) {
    PID_controller();
  } else {
    battery_read();
  }
}

float enc_vel, enc_acc;
void battery_read()
{
  M3.setDuty(0);
  float batteryVoltage = battery.getRaw()/236.0;
  Serial.print("Battery voltage: ");
  Serial.print(batteryVoltage,3);
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  angle = -euler.y();
  Serial.print(", Angle: ");
  Serial.println(angle, 2);
  delay(3000);
}
void PID_controller()
{
  derive(Ts,encoder1.getRawCount(),enc_vel, enc_acc);
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
  float frict = dir_PID * (friction (enc_vel) );
  float fill = PID + frict + correction;
  Serial.print(angle, 4); Serial.print(",");
  Serial.print(velocity, 4); Serial.print(",");
  Serial.print(vel_wheel, 4); Serial.print(",");
  float servo_angle = constrain(100 + PID/100.00, 0, 180);
  // servo2.setAngle(int(servo_angle));
  fill = constrain(fill, -255, 255);
  Serial.println(fill, 2);
  if(abs(angle) < 0.4) M3.setFill(fill);
  else M3.setFill(0);
  
}