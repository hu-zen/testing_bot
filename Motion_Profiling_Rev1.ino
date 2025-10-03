//=======================================================================================================
// KONTROL MOTOR DENGAN MOTION PROFILING VERSI INTEGER (FINAL & STABIL)
//
// Penulis: [Nama Anda]
// Deskripsi: Menggabungkan keandalan program integer sebelumnya dengan metodologi motion profiling.
//            Menggunakan matematika integer sepenuhnya untuk menghindari eror floating-point dan
//            memastikan robot berhenti dengan sempurna. Dilengkapi dengan timeout /cmd_vel.
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

// --- Variabel Target & Aktual (semua integer) ---
int target_left_speed = 0;
int target_right_speed = 0;
int current_left_speed = 0;
int current_right_speed = 0;

// --- Parameter Motion Profiling (Integer) ---
const int MAX_ACCELERATION = 400; // Satuan: PWM per detik.
const int MAX_SPEED = 500;

// --- Variabel Waktu ---
unsigned long lastUpdateTime = 0;
unsigned long lastCmdVelTime = 0;

// --- Parameter Timeout ---
const int CMD_VEL_TIMEOUT = 500; // dalam milidetik

//===============================================================================
// --- Callback Function ---
//===============================================================================
void twistCallback(const geometry_msgs::Twist& twist_msg) {
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
  
  lastUpdateTime = micros();
  lastCmdVelTime = millis();
}

//===============================================================================
// --- Loop Utama ---
//===============================================================================
void loop() {
  nh.spinOnce();

  // --- Mekanisme Timeout ---
  if (millis() - lastCmdVelTime > CMD_VEL_TIMEOUT) {
    target_left_speed = 0;
    target_right_speed = 0;
  }

  // --- LOGIKA UTAMA: MOTION PROFILING DENGAN INTEGER ---
  unsigned long currentTime = micros();
  unsigned long elapsedTime = currentTime - lastUpdateTime;

  // Lakukan update hanya pada interval yang cukup untuk menghindari perhitungan yang tidak perlu
  // 10000 microseconds = 10ms = 100Hz
  if (elapsedTime >= 10000) {
    lastUpdateTime = currentTime;
    
    // Konversi elapsedTime ke detik (sebagai float hanya untuk kalkulasi ini)
    float elapsedTimeSec = elapsedTime / 1000000.0;
    
    // Hitung perubahan kecepatan maksimum yang diizinkan dalam interval ini
    int max_speed_change = (int)(MAX_ACCELERATION * elapsedTimeSec);
    
    // Pastikan perubahan minimal 1 jika akselerasi diinginkan
    if (max_speed_change == 0) max_speed_change = 1;

    // --- Proses untuk Motor Kiri ---
    if (current_left_speed < target_left_speed) {
      current_left_speed += max_speed_change;
      if (current_left_speed > target_left_speed) {
        current_left_speed = target_left_speed;
      }
    } else if (current_left_speed > target_left_speed) {
      current_left_speed -= max_speed_change;
      if (current_left_speed < target_left_speed) {
        current_left_speed = target_left_speed;
      }
    }

    // --- Proses untuk Motor Kanan ---
    if (current_right_speed < target_right_speed) {
      current_right_speed += max_speed_change;
      if (current_right_speed > target_right_speed) {
        current_right_speed = target_right_speed;
      }
    } else if (current_right_speed > target_right_speed) {
      current_right_speed -= max_speed_change;
      if (current_right_speed < target_right_speed) {
        current_right_speed = target_right_speed;
      }
    }

    motorControlL.moveAtSpeed(current_left_speed);
    motorControlR.moveAtSpeed(current_right_speed);
  }
}
