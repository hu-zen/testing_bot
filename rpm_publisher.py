#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int16, Float32
import time

# --- Konstanta Fisik Robot (Salin dari rpm_speed_log.py Anda) ---
# Pastikan nilai-nilai ini sama persis dengan yang ada di skrip asli Anda
TICKS_PER_REVOLUTION = 76.433121 
WHEEL_RADIUS = 0.15
#--------------------------------------------------------------------

# Variabel global untuk menyimpan data
last_time_right = None
last_ticks_right = 0
last_time_left = None
last_ticks_left = 0

# --- BAGIAN BARU: Membuat Publisher untuk RQT Plot ---
# Kita akan membuat dua topic baru khusus untuk data RPM
rpm_right_pub = rospy.Publisher('/rpm/right_wheel', Float32, queue_size=10)
rpm_left_pub = rospy.Publisher('/rpm/left_wheel', Float32, queue_size=10)
# ----------------------------------------------------

def right_ticks_callback(msg):
    global last_time_right, last_ticks_right

    current_time = time.time()
    current_ticks = msg.data

    if last_time_right is not None:
        delta_time = current_time - last_time_right
        delta_ticks = current_ticks - last_ticks_right

        # Menghindari pembagian dengan nol
        if delta_time > 0:
            # Kalkulasi RPM (Revolutions Per Minute)
            rpm = (delta_ticks / TICKS_PER_REVOLUTION) / (delta_time / 60.0)
            
            # --- BAGIAN BARU: Publish data RPM ---
            rpm_right_pub.publish(rpm)
            # -------------------------------------
            
            rospy.loginfo(f"RPM Right Wheel: {rpm}")

    last_time_right = current_time
    last_ticks_right = current_ticks

def left_ticks_callback(msg):
    global last_time_left, last_ticks_left

    current_time = time.time()
    current_ticks = msg.data

    if last_time_left is not None:
        delta_time = current_time - last_time_left
        delta_ticks = current_ticks - last_ticks_left

        if delta_time > 0:
            rpm = (delta_ticks / TICKS_PER_REVOLUTION) / (delta_time / 60.0)
            
            # --- BAGIAN BARU: Publish data RPM ---
            rpm_left_pub.publish(rpm)
            # -------------------------------------

            rospy.loginfo(f"RPM Left Wheel: {rpm}")

    last_time_left = current_time
    last_ticks_left = current_ticks

if __name__ == '__main__':
    rospy.init_node('rpm_publisher_node')
    
    # Subscriber ini tetap sama, mendengarkan data mentah dari Arduino
    rospy.Subscriber('right_ticks', Int16, right_ticks_callback)
    rospy.Subscriber('left_ticks', Int16, left_ticks_callback)
    
    rospy.loginfo("RPM Publisher Node is running...")
    rospy.spin()
