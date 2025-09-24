//=============================Motor Controll with Direct Cycle PWM (DCP)===============================
// Versi 6.0 - Setiap perintah joystick memicu siklus PWM untuk deselerasi yang sangat cepat.
//======================================================================================================

#include <Servo.h>
#include <HB25MotorControl.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>

//===========================================================================================
// --- BAGIAN KALIBRASI (UBAH NILAI DI SINI UNTUK MENGATUR PENGEREMAN) ---
//===========================================================================================
// 1. Mengatur jumlah TAHAPAN dalam siklus pengereman. 
//    Angka lebih besar = pengereman lebih HALUS namun sedikit lebih lambat.
const int DECEL_STAGES = 4; 

// 2. Mengatur jeda antar TAHAPAN dalam MIKRODETIK.
//    Angka lebih besar = pengereman lebih LAMBAT.
const int STAGE_DELAY_US = 150; // 150 mikrodetik = 0.00015 detik
//===========================================================================================


const byte controlPinL = 6;
const byte controlPinR = 7;

HB25MotorControl motorControlL(controlPinL);
HB25MotorControl motorControlR(controlPinR);

ros::NodeHandle nh;

// "Memori" untuk menyimpan kecepatan terakhir
int current_left_speed = 0;
int current_right_speed = 0;

void twistCallback(const geometry_msgs::Twist& twist_msg) {
  // Kalkulasi kecepatan target dari ROS (tidak diubah)
  int linear_speed = (int)(twist_msg.linear.x * 300);
  int angular_speed = (int)(twist_msg.angular.z * 300);
  int left_speed = linear_speed - angular_speed;
  int right_speed = linear_speed + angular_speed;

  left_speed = constrain(left_speed, -500, 500);
  right_speed = constrain(right_speed, -500, 500);

  // --- LOGIKA DIRECT CYCLE PWM (DCP) ---

  // Untuk Motor Kiri
  if (left_speed == 0 && current_left_speed != 0) {
    // Jika perintahnya BERHENTI dan motor SEDANG BERGERAK...
    // Jalankan siklus pengereman.
    for (int i = DECEL_STAGES - 1; i >= 0; i--) {
      int pwm_step = (current_left_speed * i) / DECEL_STAGES;
      motorControlL.moveAtSpeed(pwm_step);
      delayMicroseconds(STAGE_DELAY_US);
    }
  } else {
    // Jika tidak, jalankan perintah secara langsung.
    motorControlL.moveAtSpeed(left_speed);
  }

  // Untuk Motor Kanan
  if (right_speed == 0 && current_right_speed != 0) {
    for (int i = DECEL_STAGES - 1; i >= 0; i--) {
      int pwm_step = (current_right_speed * i) / DECEL_STAGES;
      motorControlR.moveAtSpeed(pwm_step);
      delayMicroseconds(STAGE_DELAY_US);
    }
  } else {
    motorControlR.moveAtSpeed(right_speed);
  }

  // Update "memori" kecepatan dengan target yang baru
  current_left_speed = left_speed;
  current_right_speed = right_speed;
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &twistCallback);

void setup() {
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(sub);
  motorControlL.begin();
  motorControlR.begin();
}

void loop() {
  nh.spinOnce();
}
