#!/usr/bin/env python3
import rospy
import matplotlib.pyplot as plt
from sensor_msgs.msg import Imu
from matplotlib.animation import FuncAnimation
import time
import numpy as np
from collections import deque
import threading

lock = threading.Lock()

imu_topic  = '/livox/imu'   #/bhr_b3/imu  /livox/imu
plane_topic = '/test/foot_dist2hyp'
contact_topic = '/test/contact'

imu_data = {
    'time': deque(maxlen = 10000),
    'accel_x': deque(maxlen = 10000),
    'accel_y': deque(maxlen = 10000),
    'accel_z': deque(maxlen = 10000)
}

contact_data = {
    'time': deque(maxlen = 11000),
    'contact': deque(maxlen = 11000)
}

footdist_data = {
    'time': deque(maxlen = 11000),
    'footl_dist': deque(maxlen = 11000),
    'footr_dist': deque(maxlen = 11000)
}

start_time = None
last_update_time = None
def imu_callback(imu_msg):
    global imu_data, start_time, last_update_time
    if start_time is None:
        rospy.loginfo("Check IMU topic...")
        start_time = time.time()
        return
    
    time_elapsed = time.time() - start_time
    accel_x = imu_msg.linear_acceleration.x
    # accel_x = 0
    accel_y = imu_msg.linear_acceleration.y
    accel_z = imu_msg.linear_acceleration.z
    
    with lock:
        imu_data['time'].append(time_elapsed)
        imu_data['accel_x'].append(accel_x)
        imu_data['accel_y'].append(accel_y)
        imu_data['accel_z'].append(accel_z)
    
    last_update_time = time.time()

def contact_callback(contact_msg):
    global contact_data, start_time
    if start_time is None:
        rospy.logwarn("Check IMU topic in RealTimePlot.py...")
        return 
    time_elapsed = time.time() - start_time
    contact = contact_msg.linear_acceleration.x
    with lock:
        contact_data['time'].append(time_elapsed)
        contact_data['contact'].append(contact)

def foot_callback(foot_msg):
    global footdist_data, start_time
    if start_time is None:
        rospy.logwarn("Check IMU topic in RealTimePlot.py...")
        return 
    time_elapsed = time.time() - start_time
    footl_dist = foot_msg.linear_acceleration.x
    footr_dist = foot_msg.linear_acceleration.y
    
    with lock:
        footdist_data['time'].append(time_elapsed)
        footdist_data['footl_dist'].append(footl_dist)
        footdist_data['footr_dist'].append(footr_dist)

# Init ROS
rospy.init_node('plotter')
rospy.Subscriber(imu_topic, Imu, imu_callback)
rospy.Subscriber(plane_topic, Imu, foot_callback)
rospy.Subscriber(contact_topic, Imu, contact_callback)

# plot
fig, (ax1, ax2, ax3) = plt.subplots(3, 1, sharex=True, figsize=(5, 4))
plt.subplots_adjust(top=0.924, bottom=0.136, left=0.133, right=0.96, hspace=0.345, wspace=0.2)
ax1.set_title('Inertial Data', fontsize = '10')
if imu_topic == '/livox/imu':
    ax1.set_ylabel('Acc (g)', fontsize = '10')
else:
    ax1.set_ylabel('Acc (m/s^2)', fontsize = '10')
line1, = ax1.plot([], [], '#D95319', label='X')
line2, = ax1.plot([], [], '#F0AD25', label='Y')
line3, = ax1.plot([], [], '#0072BC', label='Z')
ax1.set_ylim(-1, 1)
ax1.legend(loc='upper right')

ax2.set_title('Foot Clearance', fontsize = '10')
ax2.set_ylabel('Dist (m)', fontsize = '10')
line4, = ax2.plot([], [], '#0072BC', label='footL')
line5, = ax2.plot([], [], '#D95319', label='footR')
ax2.set_ylim(-1, 1)
ax2.legend(loc='upper right')

ax3.set_title('Contact Events', fontsize = '10')
ax3.set_xlabel('Time (s)', fontsize = '10')
ax3.set_ylabel('L/R (1/2)')
line6, = ax3.plot([], [], '#D95319', label='contact')
ax3.set_ylim(-1, 1)
ax3.legend(loc='upper right')

# display the recent 10 s
time_window = 10
def update(frame):
    global imu_data, footdist_data, contact_data, start_time, last_update_time
    if start_time is None:
        return line1, line2, line3, line4, line5, line6
    
    if last_update_time is not None and time.time() - last_update_time > 0.5:  # If no data is received within 0.5 seconds
        return line1, line2, line3, line4, line5, line6  # stop update plot
    
    current_time = time.time() - start_time

    with lock:
        time_indices_imu = [i for i, t in enumerate(imu_data['time']) if current_time - time_window <= t <= current_time]
        time_indices_footdist = [i for i, t in enumerate(footdist_data['time']) if current_time - time_window <= t <= current_time]
        time_indices_contact = [i for i, t in enumerate(contact_data['time']) if current_time - time_window <= t <= current_time]
    
    if time_indices_imu:
        time_slice = [imu_data['time'][i] - (current_time - time_window/2) for i in time_indices_imu]
        accel_x_slice = [imu_data['accel_x'][i] for i in time_indices_imu]
        accel_y_slice = [imu_data['accel_y'][i] for i in time_indices_imu]
        accel_z_slice = [imu_data['accel_z'][i] for i in time_indices_imu]
        
        line1.set_data(time_slice, accel_x_slice)
        line2.set_data(time_slice, accel_y_slice)
        line3.set_data(time_slice, accel_z_slice)
    
    footl_slice = [0]
    footr_slice = [0]
    if time_indices_footdist:
        time_slice = [footdist_data['time'][i] - (current_time - time_window/2) for i in time_indices_footdist]
        footl_slice = [footdist_data['footl_dist'][i] for i in time_indices_footdist]
        footr_slice = [footdist_data['footr_dist'][i] for i in time_indices_footdist]
        
        line4.set_data(time_slice, footl_slice)
        line5.set_data(time_slice, footr_slice)
        
    contact_slice = [0]
    if time_indices_contact:
        time_slice = [contact_data['time'][i] - (current_time - time_window/2) for i in time_indices_contact]
        contact_slice = [contact_data['contact'][i] for i in time_indices_contact]
        
        line6.set_data(time_slice, contact_slice)
    
    ax1.set_xlim(-time_window/2, time_window/2)
    ax2.set_xlim(-time_window/2, time_window/2)
    ax3.set_xlim(-time_window/2, time_window/2)
    ax1.tick_params(axis='x', labelsize=8)
    ax2.tick_params(axis='x', labelsize=8)
    ax3.tick_params(axis='x', labelsize=8)
    
    ax1.set_xticks(np.linspace(-time_window/2, time_window/2, 5))
    ax1.set_xticklabels([f"{current_time - time_window/2 + i*(time_window/4):.2f}" for i in range(5)])
    
    if time_indices_imu:
        ax1.set_ylim(min(min(accel_x_slice), min(accel_y_slice), min(accel_z_slice)) - 1,
                      max(max(accel_x_slice), max(accel_y_slice), max(accel_z_slice)) + 1)
    
    if time_indices_footdist:
        ax2.set_ylim(min(min(footl_slice), min(footr_slice)) - 0.05,
                      max(max(footl_slice), max(footr_slice)) + 0.05)
                   
    if time_indices_contact:
        ax3.set_ylim(min(contact_slice)-0.5, max(contact_slice)+0.5)

    fig.canvas.draw_idle()

    return line1, line2, line3, line4, line5, line6

ani = FuncAnimation(fig, update, interval=100, blit=False, save_count=1000)
fig.canvas.manager.window.setWindowTitle("Data Visualization")
plt.show()
rospy.spin()