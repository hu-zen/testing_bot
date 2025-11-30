//rostopic pub -1 /set_mode std_msgs/Int8 "data: 1"
//rostopic pub -1 /set_mode std_msgs/Int8 "data: 0"
//=======================================================================================================
// PROGRAM KONTROL MOTOR VERSI FINAL (MENGGUNAKAN std_msgs::Int8)
//
// PERUBAHAN UTAMA:
// 1. Mengganti std_msgs::Bool dengan std_msgs::Int8 untuk menghindari error kompilasi.
// 2. Baud rate tetap 115200 agar sesuai dengan controller.launch Anda.
// 3. Tidak ada lagi perintah Serial.print().
//
// ATURAN MODE:
// - Kirim angka 1 ke /set_mode -> Mode Halus (control_pwm)
// - Kirim angka 0 ke /set_mode -> Mode Langsung (hb25_rosserial)
//=======================================================================================================

#include <Servo.h>
#include <HB25MotorControl.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int8.h> // MENGGANTI std_msgs/Bool.h

// --- Konfigurasi Pin ---
const byte controlPinL = 6;
const byte controlPinR = 7;

HB25MotorControl motorControlL(controlPinL);
HB25MotorControl motorControlR(controlPinR);

// --- Inisialisasi Node ROS ---
ros::NodeHandle nh;

// --- Variabel untuk Kontrol Halus ---
int target_left_speed = 0;
int target_right_speed = 0;
int current_left_speed = 0;
int current_right_speed = 0;
unsigned long lastMotorUpdateTime = 0;
const int MOTOR_UPDATE_INTERVAL = 20;
const int SPEED_STEP = 5;

// --- Variabel Status Mode ---
bool isSmoothMode = false; // Default ke Mode Langsung (0)

//===============================================================================
// --- Callback untuk Perintah Kecepatan ---
//===============================================================================
void twistCallback(const geometry_msgs::Twist& twist_msg) {
  int linear = (int)(twist_msg.linear.x * 300);
  int angular = (int)(twist_msg.angular.z * 300);

  target_left_speed = constrain(linear - angular, -500, 500);
  target_right_speed = constrain(linear + angular, -500, 500);

  if (!isSmoothMode) { // Jika Mode Langsung
    motorControlL.moveAtSpeed(target_left_speed);
    motorControlR.moveAtSpeed(target_right_speed);
  }
}

//===============================================================================
// --- Callback untuk Mengganti Mode (MENGGUNAKAN Int8) ---
//===============================================================================
void modeCallback(const std_msgs::Int8& msg) {
  isSmoothMode = (msg.data == 1); // Jika data adalah 1, mode halus = true. Selain itu, false.
  
  // Reset kecepatan saat beralih mode untuk keamanan
  target_left_speed = 0;
  target_right_speed = 0;
  current_left_speed = 0;
  current_right_speed = 0;
  motorControlL.moveAtSpeed(0);
  motorControlR.moveAtSpeed(0);
}

// --- Deklarasi Subscriber (MENGGUNAKAN Int8) ---
ros::Subscriber<geometry_msgs::Twist> sub_twist("cmd_vel", &twistCallback);
ros::Subscriber<std_msgs::Int8> sub_mode("/set_mode", &modeCallback);

//===============================================================================
// --- Setup ---
//===============================================================================
void setup() {
  // Atur Baud Rate ROS agar SAMA PERSIS dengan di controller.launch
  nh.getHardware()->setBaud(115200);
  
  nh.initNode();
  nh.subscribe(sub_twist);
  nh.subscribe(sub_mode);

  motorControlL.begin();
  motorControlR.begin();
}

//===============================================================================
// --- Loop Utama ---
//===============================================================================
void loop() {
  nh.spinOnce();

  // Logika untuk Mode Halus
  if (isSmoothMode) {
    unsigned long currentTime = millis();
    if (currentTime - lastMotorUpdateTime >= MOTOR_UPDATE_INTERVAL) {
      lastMotorUpdateTime = currentTime;

      if (current_left_speed < target_left_speed) current_left_speed = min(current_left_speed + SPEED_STEP, target_left_speed);
      else if (current_left_speed > target_left_speed) current_left_speed = max(current_left_speed - SPEED_STEP, target_left_speed);

      if (current_right_speed < target_right_speed) current_right_speed = min(current_right_speed + SPEED_STEP, target_right_speed);
      else if (current_right_speed > target_right_speed) current_right_speed = max(current_right_speed - SPEED_STEP, target_right_speed);

      motorControlL.moveAtSpeed(current_left_speed);
      motorControlR.moveAtSpeed(current_right_speed);
    }
  }
}
