#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from math import sqrt, atan2, sin, cos, pi, exp
import math
from tf.transformations import euler_from_quaternion

global dis ,an_diff, angle_diff, angle, current_pitch, current_roll, current_yaw
y = []

def pose_twist_callback(odom_msg):
    global dis, angle_diff, angle, current_pitch, current_roll, current_yaw
    pose = odom_msg.pose.pose
    pose.position.x = round(pose.position.x, 4)
    pose.position.y = round(pose.position.y, 4)
    goal_pose = goal

    dis = sqrt(pow((goal_pose.position.x - pose.position.x), 2) + pow((goal_pose.position.y - pose.position.y), 2))
    current_roll, current_pitch, current_yaw = euler_from_quaternion([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
    angle = atan2(goal_pose.position.y - pose.position.y, goal_pose.position.x - pose.position.x) - current_yaw
    if angle > pi:
        angle -= 2 * pi
    elif angle < -pi:
        angle += 2 * pi  # Angle for avoiding infinity
    angle_diff = min(0.4, 0.45 * abs(angle))*(1 if angle>0 else -1)
    angle_diff = -abs(angle_diff) if angle>2*pi/3 else angle_diff


def avoid_obs(msg):
    global y
    y = [1.65 if v>=1.65  or math.isinf(v) else v for v in msg.ranges]

def avoid_obstacle():
    global dis_1
    val_max_1 = min(y[0:45])
    val_max_2 = min(y[315:360])
    val_min_i = y.index(min(val_max_1, val_max_2)) 

    try:  
        x1 = (val_max_1 - val_max_2) / abs(val_max_1 - val_max_2)
    except ZeroDivisionError:
        x1 = 0

    val_max_a = (val_min_i + (0.1*x1)) % len(y)
    dis_1 = min(1.65 if v>= 1.65 or math.isinf(v) else v for v in y[315:] + y[:46])
    angle_diff_1 = atan2(sin(val_max_a - val_min_i), cos(val_max_a - val_min_i))


    k_goal = dis/orig_dis
    k_ang = 4 / (1 + exp(-5 * (dis_1 - 1.65)))
    k_obs = abs(7.389 - exp(k_ang))
    v1 = 4 * (1-exp(exp(1-(dis_1/1.65))-1.65)) * k_goal
    v2 = 0.6*k_ang*angle_diff +  1.8 * k_obs * angle_diff_1
    print(k_ang)

    vel.linear.x = min(0.5,abs(v1)) * (1 if v1>0 else -1)
    vel.angular.z = v2
    vel.angular.z = min(0.4, abs(vel.angular.z)) * (1 if vel.angular.z > 0 else -1)  # Limiting the resultant angular velocity
    pub.publish(vel)

if __name__ == '__main__':
    rospy.init_node('fin_g2g', anonymous=True)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    sub_odom = rospy.Subscriber('/odom', Odometry, pose_twist_callback)
    sub_scan = rospy.Subscriber('/scan', LaserScan, avoid_obs)
    goal = Pose()
    pose = Pose()
    goal.position.x = float(input('Enter x coordinate:'))
    goal.position.y = float(input('Enter y coordinate:'))
    orig_dis = sqrt(pow((goal.position.x - pose.position.x), 2) + pow((goal.position.y - pose.position.y), 2))
    
    vel = Twist()

    rospy.wait_for_message('/odom', Odometry)

    while dis > 0.1 and not rospy.is_shutdown():
        avoid_obstacle()
        rospy.sleep(0.05)

    vel.linear.x = 0
    vel.angular.z = 0
    pub.publish(vel)
    rospy.spin()
