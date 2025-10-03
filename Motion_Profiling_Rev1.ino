//=======================================================================================================
// PROGRAM KONTROL MOTOR UNTUK PENELITIAN SKRIPSI (DUAL MODE: DIRECT vs PROFILING STABIL)
//
// FITUR:
// 1. Menggunakan tipe pesan std_msgs/Int8 yang sudah terbukti bekerja di environment Anda.
// 2. Mode 0: Kontrol Langsung (hb25_rosserial) sebagai baseline.
// 3. Mode 1: Motion Profiling versi Integer yang stabil dan responsif untuk joystick.
// 4. TIDAK menggunakan timeout yang menyebabkan gerakan tersendak.
//
// ATURAN MODE (via topik /set_mode):
// - Kirim angka 1 -> Mode 1: Motion Profiling (Gerak Halus & Stabil)
// - Kirim angka 0 -> Mode 0: Kontrol Langsung (Gerak Cepat & Responsif)
//=======================================================================================================

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

// --- Variabel Kecepatan (semua integer) ---
int target_left_speed = 0;
int target_right_speed = 0;
int current_left_speed = 0;
int current_right_speed = 0;

// --- Parameter & Variabel Waktu ---
unsigned long lastUpdateTime = 0;
const int UPDATE_INTERVAL_MS = 10; // Update setiap 10ms (100Hz)

// --- Parameter Motion Profiling ---
const int MAX_ACCELERATION = 800; // Satuan: PWM per detik.
const int MAX_SPEED = 500;
int speed_step; // Dihitung di setup()

// --- Variabel Status Mode ---
// false = Mode Langsung (0), true = Mode Motion Profiling (1)
bool useMotionProfiling = false; 

//===============================================================================
// --- Callback untuk Perintah Kecepatan ---
//===============================================================================
void twistCallback(const geometry_msgs::Twist& twist_msg) {
  int linear = (int)(twist_msg.linear.x * 300);
  int angular = (int)(twist_msg.angular.z * 300);
  
  // Selalu hitung dan simpan kecepatan target
  target_left_speed = constrain(linear - angular, -MAX_SPEED, MAX_SPEED);
  target_right_speed = constrain(linear + angular, -MAX_SPEED, MAX_SPEED);

  // Jika dalam Mode Langsung, langsung eksekusi perintah motor
  if (!useMotionProfiling) {
    motorControlL.moveAtSpeed(target_left_speed);
    motorControlR.moveAtSpeed(target_right_speed);
  }
  // Jika mode Motion Profiling, nilai target akan diproses di loop() utama
}

//===============================================================================
// --- Callback untuk Mengganti Mode ---
//===============================================================================
void modeCallback(const std_msgs::Int8& msg) {
  useMotionProfiling = (msg.data == 1);
  // Reset semua variabel kecepatan saat beralih mode untuk transisi yang aman
  target_left_speed = 0;
  target_right_speed = 0;
  current_left_speed = 0;
  current_right_speed = 0;
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
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(sub_twist);
  nh.subscribe(sub_mode);

  motorControlL.begin();
  motorControlR.begin();
  
  // Hitung speed_step berdasarkan parameter profiling
  speed_step = (int)(MAX_ACCELERATION * (UPDATE_INTERVAL_MS / 1000.0));
  if (speed_step == 0) {
    speed_step = 1;
  }
}

//===============================================================================
// --- Loop Utama ---
//===============================================================================
void loop() {
  nh.spinOnce();

  // Logika Motion Profiling hanya berjalan jika modenya aktif
  if (useMotionProfiling) {
    unsigned long currentTime = millis();
    if (currentTime - lastUpdateTime >= UPDATE_INTERVAL_MS) {
      lastUpdateTime = currentTime;

      // Gunakan logika if/else yang andal untuk "merayap" ke target
      if (current_left_speed < target_left_speed) {
        current_left_speed = min(current_left_speed + speed_step, target_left_speed);
      } else if (current_left_speed > target_left_speed) {
        current_left_speed = max(current_left_speed - speed_step, target_left_speed);
      }

      if (current_right_speed < target_right_speed) {
        current_right_speed = min(current_right_speed + speed_step, target_right_speed);
      } else if (current_right_speed > target_right_speed) {
        current_right_speed = max(current_right_speed - speed_step, target_right_speed);
      }

      motorControlL.moveAtSpeed(current_left_speed);
      motorControlR.moveAtSpeed(current_right_speed);
    }
  }
}
