//=======================================================================================================
// PROGRAM KONTROL MOTOR UNTUK PENELITIAN SKRIPSI (DUAL MODE: RAMPING vs PROFILING)
//
// FITUR:
// 1. Beralih antara dua metode kontrol yang andal menggunakan matematika integer.
// 2. Dilengkapi timeout /cmd_vel yang dapat disesuaikan untuk keamanan.
//
// ATURAN MODE (via topik /set_mode, tipe std_msgs/Int8):
// - Kirim angka 1 -> Mode 1: Motion Profiling (Metode Baru untuk Skripsi)
// - Kirim angka 0 -> Mode 0: Ramping / Smoothing (Metode Lama yang Stabil)
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
unsigned long lastCmdVelTime = 0;
const int UPDATE_INTERVAL_MS = 10; // Update setiap 10ms (100Hz)

// --- Parameter Keamanan ---
// NAIKKAN NILAI INI jika robot terasa tersendak saat menggunakan joystick
// Contoh: 1000 (1 detik)
const int CMD_VEL_TIMEOUT = 500; // dalam milidetik

// --- Parameter untuk Setiap Mode ---
// Mode 0: Ramping (dari program favorit Anda)
const int SPEED_STEP = 2; 
// Mode 1: Motion Profiling
const int MAX_ACCELERATION = 400; // Satuan: PWM per detik.

// --- Variabel Status Mode ---
// false = Mode Ramping (0), true = Mode Motion Profiling (1)
bool useMotionProfiling = false; 

//===============================================================================
// --- Callbacks ---
//===============================================================================
void twistCallback(const geometry_msgs::Twist& twist_msg) {
  lastCmdVelTime = millis();
  int linear = (int)(twist_msg.linear.x * 300);
  int angular = (int)(twist_msg.angular.z * 300);
  
  target_left_speed = constrain(linear - angular, -500, 500);
  target_right_speed = constrain(linear + angular, -500, 500);
}

void modeCallback(const std_msgs::Int8& msg) {
  useMotionProfiling = (msg.data == 1);
  // Reset kecepatan saat beralih mode untuk keamanan
  target_left_speed = 0; target_right_speed = 0;
  current_left_speed = 0; current_right_speed = 0;
  motorControlL.moveAtSpeed(0); motorControlR.moveAtSpeed(0);
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
  lastUpdateTime = millis();
  lastCmdVelTime = millis();
}

//===============================================================================
// --- Loop Utama ---
//===============================================================================
void loop() {
  nh.spinOnce();

  if (millis() - lastCmdVelTime > CMD_VEL_TIMEOUT) {
    target_left_speed = 0;
    target_right_speed = 0;
  }

  if (millis() - lastUpdateTime >= UPDATE_INTERVAL_MS) {
    lastUpdateTime = millis();

    if (useMotionProfiling) {
      // --- MODE 1: MOTION PROFILING (INTEGER) ---
      int max_speed_change = (int)(MAX_ACCELERATION * (UPDATE_INTERVAL_MS / 1000.0));
      if (max_speed_change == 0) max_speed_change = 1;

      if (current_left_speed < target_left_speed) current_left_speed = min(current_left_speed + max_speed_change, target_left_speed);
      else if (current_left_speed > target_left_speed) current_left_speed = max(current_left_speed - max_speed_change, target_left_speed);

      if (current_right_speed < target_right_speed) current_right_speed = min(current_right_speed + max_speed_change, target_right_speed);
      else if (current_right_speed > target_right_speed) current_right_speed = max(current_right_speed - max_speed_change, target_right_speed);

    } else {
      // --- MODE 0: RAMPING / SMOOTHING (INTEGER) ---
      if (current_left_speed < target_left_speed) current_left_speed = min(current_left_speed + SPEED_STEP, target_left_speed);
      else if (current_left_speed > target_left_speed) current_left_speed = max(current_left_speed - SPEED_STEP, target_left_speed);

      if (current_right_speed < target_right_speed) current_right_speed = min(current_right_speed + SPEED_STEP, target_right_speed);
      else if (current_right_speed > target_right_speed) current_right_speed = max(current_right_speed - SPEED_STEP, target_right_speed);
    }
    
    motorControlL.moveAtSpeed(current_left_speed);
    motorControlR.moveAtSpeed(current_right_speed);
  }
}
