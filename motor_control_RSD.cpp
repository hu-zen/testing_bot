//=============================Motor Controll with Tunable RSD===============================
// Versi 5.1 - Menambahkan parameter yang mudah diubah untuk kalibrasi deselerasi.
//===========================================================================================

#include <Servo.h>
#include <HB25MotorControl.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>

//===========================================================================================
// --- BAGIAN KALIBRASI (UBAH NILAI DI SINI) ---
//===========================================================================================
// 1. Mengatur jumlah tahapan pengereman. Angka lebih kecil = lebih tajam.
const int DECEL_STAGE_DIVIDER = 3; 

// 2. Mengatur durasi jeda pengereman dalam mikrodetik. Angka lebih besar = lebih lambat.
const int DECEL_DELAY_US = 200;
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
  int linear_speed = (int)(twist_msg.linear.x * 300);
  int angular_speed = (int)(twist_msg.angular.z * 300);
  int left_speed = linear_speed - angular_speed;
  int right_speed = linear_speed + angular_speed;

  left_speed = constrain(left_speed, -500, 500);
  right_speed = constrain(right_speed, -500, 500);

  // --- LOGIKA RAPID STAGED DECELERATION (RSD) ---

  // Untuk Motor Kiri
  if (left_speed == 0 && current_left_speed != 0) {
    motorControlL.moveAtSpeed(current_left_speed / DECEL_STAGE_DIVIDER);
    delayMicroseconds(DECEL_DELAY_US);
    motorControlL.moveAtSpeed(0);
  } else {
    motorControlL.moveAtSpeed(left_speed);
  }

  // Untuk Motor Kanan
  if (right_speed == 0 && current_right_speed != 0) {
    motorControlR.moveAtSpeed(current_right_speed / DECEL_STAGE_DIVIDER);
    delayMicroseconds(DECEL_DELAY_US);
    motorControlR.moveAtSpeed(0);
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
