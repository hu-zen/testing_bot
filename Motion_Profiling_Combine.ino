/*
// ATURAN MODE (via topik /set_mode):
// - Kirim angka 1 -> Mode Motion Profiling (Trapezoidal)
// - Kirim angka 0 -> Mode Kontrol Langsung (Tanpa Profiling)
rostopic pub -1 /set_mode std_msgs/Int8 "data: 1"
rostopic pub -1 /set_mode std_msgs/Int8 "data: 0"
*/

#include <Servo.h>
#include <HB25MotorControl.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int8.h>

// --- Konfigurasi Pin ---
const byte controlPinL = 6;
const byte controlPinR = 7;

HB25MotorControl motorControlL(controlPinL);
HB25MotorControl motorControlR(controlPinR);

// --- Inisialisasi Node ROS ---
ros::NodeHandle nh;

// --- Variabel Target & Aktual ---
int target_left_speed = 0;
int target_right_speed = 0;
float current_left_speed = 0.0; // Gunakan float untuk presisi kalkulasi
float current_right_speed = 0.0;

// --- Variabel Waktu ---
unsigned long lastUpdateTime = 0;

// --- Parameter Motion Profiling (Dapat Disesuaikan untuk Penelitian) ---
const float MAX_ACCELERATION = 400.0; // Satuan: PWM per detik.
const float MAX_SPEED = 500.0;        // Kecepatan PWM maksimum.

// --- Variabel Status Mode ---
// false = Mode Langsung, true = Mode Motion Profiling
bool useMotionProfiling = false; 

//===============================================================================
// --- Callback untuk Perintah Kecepatan ---
//===============================================================================
void twistCallback(const geometry_msgs::Twist& twist_msg) {
  int linear = (int)(twist_msg.linear.x * 300);
  int angular = (int)(twist_msg.angular.z * 300);

  // Hitung dan batasi kecepatan target
  target_left_speed = constrain(linear - angular, -MAX_SPEED, MAX_SPEED);
  target_right_speed = constrain(linear + angular, -MAX_SPEED, MAX_SPEED);

  // Jika dalam Mode Langsung, langsung eksekusi perintah
  if (!useMotionProfiling) {
    motorControlL.moveAtSpeed(target_left_speed);
    motorControlR.moveAtSpeed(target_right_speed);
  }
  // Jika mode Motion Profiling, nilai target akan diproses di loop()
}

//===============================================================================
// --- Callback untuk Mengganti Mode ---
//===============================================================================
void modeCallback(const std_msgs::Int8& msg) {
  useMotionProfiling = (msg.data == 1); // Jika data adalah 1, aktifkan motion profiling
  
  // Reset semua variabel kecepatan saat beralih mode untuk transisi yang aman dan bersih
  target_left_speed = 0;
  target_right_speed = 0;
  current_left_speed = 0.0;
  current_right_speed = 0.0;
  motorControlL.moveAtSpeed(0);
  motorControlR.moveAtSpeed(0);
}

// --- Deklarasi Subscriber ---
ros::Subscriber<geometry_msgs::Twist> sub_twist("cmd_vel", &twistCallback);
ros::Subscriber<std_msgs::Int8> sub_mode("/set_mode", &modeCallback);

//===============================================================================
// --- Setup ---
//===============================================================================
void setup() {
  // Atur Baud Rate ROS agar sesuai dengan controller.launch
  nh.getHardware()->setBaud(115200);
  
  nh.initNode();
  nh.subscribe(sub_twist);
  nh.subscribe(sub_mode);

  motorControlL.begin();
  motorControlR.begin();
  
  lastUpdateTime = micros(); // Inisialisasi timer dengan presisi mikrodetik
}

//===============================================================================
// --- Loop Utama ---
//===============================================================================
void loop() {
  nh.spinOnce(); // Cek pesan masuk dari ROS

  // --- Logika hanya berjalan jika Mode Motion Profiling aktif ---
  if (useMotionProfiling) {
    // Hitung waktu yang telah berlalu dalam detik
    unsigned long currentTime = micros();
    float elapsedTime = (currentTime - lastUpdateTime) / 1000000.0;
    lastUpdateTime = currentTime;

    // Hitung perubahan kecepatan maksimum yang diizinkan sesuai aturan akselerasi
    float max_speed_change = MAX_ACCELERATION * elapsedTime;

    // Hitung error (selisih) antara target dan kecepatan saat ini
    float error_left = target_left_speed - current_left_speed;
    float error_right = target_right_speed - current_right_speed;

    // Batasi perubahan kecepatan agar tidak melebihi batas akselerasi
    float change_left = constrain(error_left, -max_speed_change, max_speed_change);
    float change_right = constrain(error_right, -max_speed_change, max_speed_change);

    // Terapkan perubahan untuk "merayap" menuju target
    current_left_speed += change_left;
    current_right_speed += change_right;

    // Kirim kecepatan yang sudah diprofilkan ke motor
    motorControlL.moveAtSpeed((int)current_left_speed);
    motorControlR.moveAtSpeed((int)current_right_speed);
  }
}
