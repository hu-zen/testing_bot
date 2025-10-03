//=======================================================================================================
// KONTROL MOTOR DENGAN MOTION PROFILING - VERSI STABIL UNTUK JOYSTICK
//
// Penulis: [Nama Anda]
// Deskripsi: Program ini mengadopsi struktur andal dari metode "ramping" sebelumnya,
//            menggunakan matematika integer dan interval update yang tetap. Fitur timeout
//            dihapus untuk memastikan responsivitas penuh saat menggunakan joystick.
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

// --- Variabel Kecepatan (semua integer, seperti program yang baik) ---
int target_left_speed = 0;
int target_right_speed = 0;
int current_left_speed = 0;
int current_right_speed = 0;

// --- Parameter & Variabel Waktu (mengikuti struktur program yang baik) ---
unsigned long lastUpdateTime = 0;
const int UPDATE_INTERVAL_MS = 10; // Update setiap 10ms (100Hz)

// --- Parameter Motion Profiling ---
const int MAX_ACCELERATION = 800; // Satuan: PWM per detik. NAIKKAN untuk akselerasi lebih cepat.
const int MAX_SPEED = 500;
int speed_step; // Akan dihitung secara dinamis di setup()

//===============================================================================
// --- Callback Function ---
// Hanya menyimpan kecepatan target, sangat cepat dan responsif.
//===============================================================================
void twistCallback(const geometry_msgs::Twist& twist_msg) {
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
  
  // --- HITUNG SPEED_STEP BERDASARKAN PARAMETER PROFILING ---
  // Ini adalah jembatan antara metodologi skripsi dan program yang andal.
  // speed_step = percepatan * durasi_interval
  speed_step = (int)(MAX_ACCELERATION * (UPDATE_INTERVAL_MS / 1000.0));
  
  // Pastikan speed_step minimal 1 agar robot bisa bergerak
  if (speed_step == 0) {
    speed_step = 1;
  }
}

//===============================================================================
// --- Loop Utama (menggunakan struktur program yang baik) ---
//===============================================================================
void loop() {
  nh.spinOnce();

  unsigned long currentTime = millis();

  if (currentTime - lastUpdateTime >= UPDATE_INTERVAL_MS) {
    lastUpdateTime = currentTime;

    // --- LOGIKA UTAMA: Menggunakan struktur if/else yang andal dari program favorit Anda ---
    
    // Untuk motor kiri
    if (current_left_speed < target_left_speed) {
      current_left_speed = min(current_left_speed + speed_step, target_left_speed);
    } else if (current_left_speed > target_left_speed) {
      current_left_speed = max(current_left_speed - speed_step, target_left_speed);
    }

    // Untuk motor kanan
    if (current_right_speed < target_right_speed) {
      current_right_speed = min(current_right_speed + speed_step, target_right_speed);
    } else if (current_right_speed > target_right_speed) {
      current_right_speed = max(current_right_speed - speed_step, target_right_speed);
    }

    // Kirim kecepatan yang sudah dihaluskan ke motor
    motorControlL.moveAtSpeed(current_left_speed);
    motorControlR.moveAtSpeed(current_right_speed);
  }
}
