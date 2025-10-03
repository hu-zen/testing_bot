//=======================================================================================================
// KONTROL MOTOR DENGAN MOTION PROFILING VERSI FINAL (DENGAN CMD_VEL TIMEOUT)
//
// Penulis: [Nama Anda]
// Deskripsi: Versi ini menambahkan mekanisme timeout untuk memastikan robot berhenti
//            ketika tidak ada perintah baru yang diterima. Ini mengatasi masalah robot
//            yang terus bergerak setelah joystick dilepas.
//=======================================================================================================

#include <Servo.h>
#include <HB25MotorControl.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>

const byte controlPinL = 6;
const byte controlPinR = 7;

HB25MotorControl motorControlL(controlPinL);
HB25MotorControl motorControlR(controlPinR);

ros::NodeHandle nh;

// --- Variabel Target & Aktual ---
int target_left_speed = 0;
int target_right_speed = 0;
float current_left_speed = 0.0;
float current_right_speed = 0.0;

// --- Parameter Motion Profiling (Dapat Disesuaikan untuk Penelitian) ---
const float MAX_ACCELERATION = 400.0; // Satuan: PWM per detik.
const float MAX_SPEED = 500.0;

// --- Variabel Waktu ---
unsigned long lastUpdateTime = 0;
unsigned long lastCmdVelTime = 0; // Waktu terakhir menerima perintah /cmd_vel

// --- Parameter Timeout (PENTING) ---
// Jika tidak ada perintah baru selama 500ms, anggap perintahnya adalah berhenti.
const int CMD_VEL_TIMEOUT = 500; // dalam milidetik

//===============================================================================
// --- Callback Function ---
//===============================================================================
void twistCallback(const geometry_msgs::Twist& twist_msg) {
  // Catat waktu setiap kali menerima pesan baru
  lastCmdVelTime = millis();

  int linear_speed = (int)(twist_msg.linear.x * 300);
  int angular_speed = (int)(twist_msg.angular.z * 300);
  
  target_left_speed = linear_speed - angular_speed;
  target_right_speed = linear_speed + angular_speed;

  target_left_speed = constrain(target_left_speed, -MAX_SPEED, MAX_SPEED);
  target_right_speed = constrain(target_right_speed, -MAX_SPEED, MAX_SPEED);
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &twistCallback);

//===============================================================================
// --- Setup ---
//===============================================================================
void setup() {
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(sub);

  motorControlL.begin();
  motorControlR.begin();
  
  // Inisialisasi timer dengan presisi yang sesuai
  lastUpdateTime = micros(); 
  lastCmdVelTime = millis();
}

//===============================================================================
// --- Loop Utama ---
//===============================================================================
void loop() {
  nh.spinOnce();

  unsigned long currentTimeMs = millis();
  
  // --- MEKANISME TIMEOUT ---
  // Jika sudah terlalu lama tidak menerima perintah, paksa target kecepatan menjadi nol.
  if (currentTimeMs - lastCmdVelTime > CMD_VEL_TIMEOUT) {
    target_left_speed = 0;
    target_right_speed = 0;
  }

  // --- LOGIKA UTAMA: MOTION PROFILING ---
  unsigned long currentTimeMicros = micros();
  float elapsedTime = (currentTimeMicros - lastUpdateTime) / 1000000.0; // Waktu dalam detik
  lastUpdateTime = currentTimeMicros;

  float max_speed_change = MAX_ACCELERATION * elapsedTime;

  // Proses untuk Motor Kiri
  float error_left = target_left_speed - current_left_speed;
  float change_left = constrain(error_left, -max_speed_change, max_speed_change);
  current_left_speed += change_left;

  // Proses untuk Motor Kanan
  float error_right = target_right_speed - current_right_speed;
  float change_right = constrain(error_right, -max_speed_change, max_speed_change);
  current_right_speed += change_right;
  
  // Deadband: jika kecepatan sangat kecil, anggap nol untuk mencegah "creep"
  if (abs(current_left_speed) < 1.0) current_left_speed = 0.0;
  if (abs(current_right_speed) < 1.0) current_right_speed = 0.0;

  // Kirim kecepatan yang sudah diprofilkan ke motor
  motorControlL.moveAtSpeed((int)current_left_speed);
  motorControlR.moveAtSpeed((int)current_right_speed);
}
