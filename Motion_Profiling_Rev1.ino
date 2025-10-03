//=======================================================================================================
// PROGRAM KONTROL MOTOR UNTUK PENELITIAN SKRIPSI (DUAL MODE: DIRECT vs PROFILING)
//
// FITUR:
// 1. Beralih antara dua metode kontrol yang andal menggunakan matematika integer.
// 2. Mode 0: Kontrol Langsung (hb25_rosserial) untuk baseline perbandingan.
// 3. Mode 1: Motion Profiling (Integer & Stabil) untuk metode yang diusulkan.
//
// ATURAN MODE (via topik /set_mode, tipe std_msgs/Int8):
// - Kirim angka 1 -> Mode 1: Motion Profiling (Gerak Halus & Terkontrol)
// - Kirim angka 0 -> Mode 0: Kontrol Langsung (Gerak Cepat & Responsif)
//=======================================================================================================
/home/iascr/Arduino/Combine_Mode_Motor/Combine_Mode_Motor.ino:84:30: error: template argument 1 is invalid
 ros::Subscriber<std_msgs/Int8> sub_mode("/set_mode", &modeCallback);
                              ^
/home/iascr/Arduino/Combine_Mode_Motor/Combine_Mode_Motor.ino: In function 'void setup()':
/home/iascr/Arduino/Combine_Mode_Motor/Combine_Mode_Motor.ino:93:24: error: no matching function for call to 'ros::NodeHandle_<ArduinoHardware, 25, 25, 280, 280>::subscribe(int&)'
   nh.subscribe(sub_mode);
                        ^
In file included from /home/iascr/Arduino/libraries/ros_lib/ros.h:38:0,
                 from /home/iascr/Arduino/Combine_Mode_Motor/Combine_Mode_Motor.ino:16:
/home/iascr/Arduino/libraries/ros_lib/ros/node_handle.h:406:8: note: candidate: bool ros::NodeHandle_<Hardware, MAX_SUBSCRIBERS, MAX_PUBLISHERS, INPUT_SIZE, OUTPUT_SIZE>::subscribe(ros::Subscriber_&) [with Hardware = ArduinoHardware; int MAX_SUBSCRIBERS = 25; int MAX_PUBLISHERS = 25; int INPUT_SIZE = 280; int OUTPUT_SIZE = 280]
   bool subscribe(Subscriber_& s)
        ^~~~~~~~~
/home/iascr/Arduino/libraries/ros_lib/ros/node_handle.h:406:8: note:   no known conversion for argument 1 from 'int' to 'ros::Subscriber_&'

exit status 1

Compilation error: template argument 1 is invalid
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
ros::Subscriber<std_msgs/Int8> sub_mode("/set_mode", &modeCallback);

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
