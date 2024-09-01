#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist, Pose
from math import sqrt, atan2, pi, exp
from tf.transformations import euler_from_quaternion

global dis, angle_diff, current_pitch, current_roll, current_yaw, sum, yaw, velocity

def pose_twist_callback(pose=NavSatFix):
    global dis, angle_diff, angle, current_pitch, current_roll, current_yaw
    pose.latitude = round(pose.latitude, 8)
    pose.longitude = round(pose.longitude, 8)
    goal_pose = goal
    #print(pose.latitude)
    dis = sqrt(pow((goal_pose.position.x - pose.latitude), 2) + pow((goal_pose.position.y - pose.latitude), 2))
    angle = atan2(goal_pose.position.y - pose.longitude, goal_pose.position.x - pose.latitude)
    #print(dis)
    #print(angle)

def yaw_callback(yaw=Imu):
    global current_pitch, current_roll, current_yaw, angle, angle_diff
    current_roll, current_pitch, current_yaw = euler_from_quaternion([yaw.orientation.x, yaw.orientation.y, yaw.orientation.z, yaw.orientation.w])

def avoid_obs(msg):
    global sum, obs_lin, goal_ang
    sum = 0
    obs_lin = 0
    goal_ang = 0

    for i in range(len(msg.ranges)):
        r = msg.ranges[i]
        r = 1.5 if r>1.5 or r==0 else r
        y = 1 - (r / 1.5)
        if i <= 45 or i >= len(msg.ranges)-45:
            sum -= 8 * y * (i if i <=45 else i - len(msg.ranges)) 
            obs_lin = exp(y) - 1.5
            goal_ang = 4 / (1 + exp(-5 * (r - 1.5)))  # Calculator for goal angular velocity
    #print(sum)

def avoid_obstacle():
    global angle, velocity, vel, angle_diff, k_lin, obs_lin, obs_ang, k_ang, goal_ang
    angle = round(angle,4)
    angle = angle - current_yaw
    if angle > pi:
        angle -= 2 * pi
    elif angle < -pi:
        angle += 2 * pi  # Angle for avoiding infinity
    
    angle_diff = min(0.4, 0.4 * abs(angle))*(1 if angle>0 else -1)
    angle_diff = -abs(angle_diff) if angle>2*pi/3 else angle_diff
    #print(current_yaw)
    #print(angle_diff)

    velocity = dis / orig_dis
    # velocity = min(0.5, abs(velocity))*(1 if velocity>0 else -1)  # Limit the velocity to 0.5

    obs_ang = abs(7.389 - exp(goal_ang)) * sum
    k_lin = 0.94 - exp(obs_lin)  # Calculator for linear velocity due to obstacle
    k_ang = 2 * obs_ang + 0.6 * goal_ang * angle_diff

    vel.linear.x = (4 * k_lin * velocity)  # Adjusting linear velocity
    vel.linear.x = min(1.5, abs(vel.linear.x)) * (1 if vel.linear.x > 0 else -1)
    vel.angular.z = k_ang
    vel.angular.z = min(1.5, abs(vel.angular.z)) * (1 if vel.angular.z > 0 else -1)  # Limiting the resultant angular velocity
    pub.publish(vel)
    #print(obs_ang)
    print(f"{vel.linear.x} {vel.angular.z}")
 
if __name__ == '__main__':
    rospy.init_node('fin_g2g', anonymous=True)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    sub_odom = rospy.Subscriber('/android/fix', NavSatFix, pose_twist_callback)
    sub_yaw = rospy.Subscriber('/android/imu', Imu, yaw_callback)
    sub_scan = rospy.Subscriber('/scan', LaserScan, avoid_obs)
    global goal
    goal = Pose()
    pose = NavSatFix()
    self_x = round(pose.latitude, 8)
    self_y=  round(pose.longitude, 8)

    goal_pose = goal

    goal.position.x = float(input('Enter x coordinate:'))
    goal.position.y = float(input('Enter y coordinate:'))

    vel = Twist()

    orig_dis = sqrt(pow((goal.position.x-self_x), 2) + pow((goal.position.y - self_y), 2))
    print(orig_dis)

    rate = rospy.Rate(20)  # 50ms = 20Hz
    while dis > 5 and not rospy.is_shutdown():
        avoid_obstacle()
        rate.sleep()
        rospy.wait_for_message('/android/fix', NavSatFix,timeout=1.5)
        
    vel.linear.x = 0
    vel.angular.z = 0
    pub.publish(vel)
    rospy.spin()
