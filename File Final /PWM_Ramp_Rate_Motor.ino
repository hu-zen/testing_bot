/*
rostopic pub -1 /set_mode std_msgs/Int8 "data: 1"
rostopic pub -1 /set_mode std_msgs/Int8 "data: 0"
ATURAN MODE:
- Kirim angka 1 ke /set_mode -> Mode Halus (control_pwm)
- Kirim angka 0 ke /set_mode -> Mode Kasar (hb25_rosserial)
*/
#include <Servo.h>
#include <HB25MotorControl.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int8.h> 

// Konfigurasi Pin
const byte controlPinL = 6;
const byte controlPinR = 7;

HB25MotorControl motorControlL(controlPinL);
HB25MotorControl motorControlR(controlPinR);

// Inisialisasi Node ROS 
ros::NodeHandle nh;

// Variabel untuk Kontrol Halus
int target_left_speed = 0;
int target_right_speed = 0;
int current_left_speed = 0;
int current_right_speed = 0;
unsigned long lastMotorUpdateTime = 0;
const int MOTOR_UPDATE_INTERVAL = 10;
const int SPEED_STEP = 2;

// Variabel Status Mode
bool isSmoothMode = true; // Default ke Mode Halus (1)

// Logika untuk mode kasar
void twistCallback(const geometry_msgs::Twist& twist_msg) {
  int linear = (int)(twist_msg.linear.x * 300);
  int angular = (int)(twist_msg.angular.z * 300);

  target_left_speed = constrain(linear - angular, -500, 500);
  target_right_speed = constrain(linear + angular, -500, 500);

  if (!isSmoothMode) { 
    motorControlL.moveAtSpeed(target_left_speed);
    motorControlR.moveAtSpeed(target_right_speed);
  }
}

void modeCallback(const std_msgs::Int8& msg) {
  isSmoothMode = (msg.data == 1); // Jika data adalah 1, mode halus = true. Selain itu, false.
  
  // Reset Ketika Ganti Mode
  target_left_speed = 0;
  target_right_speed = 0;
  current_left_speed = 0;
  current_right_speed = 0;
  motorControlL.moveAtSpeed(0);
  motorControlR.moveAtSpeed(0);
}

// Deklarasi Subscriber
ros::Subscriber<geometry_msgs::Twist> sub_twist("cmd_vel", &twistCallback);
ros::Subscriber<std_msgs::Int8> sub_mode("/set_mode", &modeCallback);

void setup() {

  nh.getHardware()->setBaud(115200);
  
  nh.initNode();
  nh.subscribe(sub_twist);
  nh.subscribe(sub_mode);

  motorControlL.begin();
  motorControlR.begin();
}


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
