//=======================================================================================================
// KONTROL MOTOR DENGAN MOTION PROFILING (TRAPEZOIDAL) UNTUK WAITER-BOT
//
// Penulis: [Nama Anda]
// Deskripsi: Program ini mengimplementasikan motion profiling trapezoidal untuk mengontrol
//            kecepatan motor. Tujuannya adalah untuk menghasilkan gerakan yang halus dan
//            terprediksi dengan mengatur akselerasi dan deselerasi secara presisi.
//            Sangat sesuai untuk "Pengembangan Kontrol Motor dengan Motion Profiling pada PWM".
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
float current_left_speed = 0.0; // Menggunakan float untuk kalkulasi yang lebih presisi
float current_right_speed = 0.0;

// --- Parameter Motion Profiling (Dapat Disesuaikan untuk Penelitian) ---
const float MAX_ACCELERATION = 400.0; // Satuan: PWM per detik. Semakin besar, semakin cepat akselerasinya.
const float MAX_SPEED = 500.0;        // Kecepatan PWM maksimum yang diizinkan.

// --- Variabel Waktu ---
unsigned long lastUpdateTime = 0;

//===============================================================================
// --- Callback Function ---
// Fungsi ini hanya bertugas untuk menyimpan kecepatan target dari ROS.
//===============================================================================
void twistCallback(const geometry_msgs::Twist& twist_msg) {
  int linear_speed = (int)(twist_msg.linear.x * 300);
  int angular_speed = (int)(twist_msg.angular.z * 300);
  
  // Simpan hasil kalkulasi ke variabel target global
  target_left_speed = linear_speed - angular_speed;
  target_right_speed = linear_speed + angular_speed;

  // Pastikan target tidak melebihi batas kecepatan maksimum
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
  
  lastUpdateTime = micros(); // Menggunakan micros() untuk presisi waktu yang lebih tinggi
}

//===============================================================================
// --- Loop Utama ---
//===============================================================================
void loop() {
  // Selalu cek pesan baru dari ROS
  nh.spinOnce();

  // --- LOGIKA UTAMA: MOTION PROFILING ---
  unsigned long currentTime = micros();
  float elapsedTime = (currentTime - lastUpdateTime) / 1000000.0; // Waktu dalam detik
  lastUpdateTime = currentTime;

  // Hitung seberapa besar kecepatan boleh berubah dalam rentang waktu ini
  float max_speed_change = MAX_ACCELERATION * elapsedTime;

  // --- Proses untuk Motor Kiri ---
  float error_left = target_left_speed - current_left_speed;
  float change_left = constrain(error_left, -max_speed_change, max_speed_change);
  current_left_speed += change_left;

  // --- Proses untuk Motor Kanan ---
  float error_right = target_right_speed - current_right_speed;
  float change_right = constrain(error_right, -max_speed_change, max_speed_change);
  current_right_speed += change_right;

  // Kirim kecepatan yang sudah diprofilkan ke motor
  motorControlL.moveAtSpeed((int)current_left_speed);
  motorControlR.moveAtSpeed((int)current_right_speed);
}
