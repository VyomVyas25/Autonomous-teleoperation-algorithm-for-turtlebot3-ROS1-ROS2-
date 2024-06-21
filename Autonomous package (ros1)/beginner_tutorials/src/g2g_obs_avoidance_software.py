#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from math import sqrt, atan2, pi, exp
from tf.transformations import euler_from_quaternion

global dis, angle_diff, angle, current_pitch, current_roll, current_yaw, sum, dis_1, max_vals, velocity

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
    global sum, dis_1, max_vals, obs_lin, goal_ang
    sum = 0
    dis_1 = []
    max_vals = []

    for i in range(len(msg.ranges)):
        r = msg.ranges[i]
        r = 1.5 if r > 1.5 else r
        y = 1 - (r / 1.5)
        dis_1.append(y)
        max_vals.append((1 - y) * 1.5)

    for i in range(len(msg.ranges)):
        if i <= 60 or i >= 300:
            sum -= 6 * dis_1[i] * (i if i <= 60 else i - 360)
   
    goal_ang=0
    obs_lin=0
    for index in list(range(0, 61)) + list(range(300, 360)):
        obs_lin = exp(dis_1[index] ) - 1.5
        goal_ang = 4 / (1 + exp(-100 * (max_vals[index] - 1.5)))  # Calculator for goal angular velocity
    

def avoid_obstacle():
    global velocity, vel, angle_diff, k_lin, obs_ang, k_ang, goal_ang
    velocity = dis / orig_dis
    velocity = min(0.5, abs(velocity))*(1 if velocity>0 else -1)  # Limit the velocity to 0.5

    obs_ang = abs(7.389 - exp(goal_ang)) * sum
    k_lin = 0.94 - exp(obs_lin)  # Calculator for linear velocity due to obstacle
    k_ang = obs_ang + 0.3 * goal_ang * angle_diff

    vel.linear.x = 3.5 * k_lin * velocity  # Adjusting linear velocity
    vel.angular.z = k_ang
    vel.angular.z = min(0.4, abs(vel.angular.z)) * (1 if vel.angular.z > 0 else -1)  # Limiting the resultant angular velocity
    pub.publish(vel)
    print(vel.angular.z)
    
if __name__ == '__main__':
    rospy.init_node('fin_g2g', anonymous=True)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    sub_odom = rospy.Subscriber('/odom', Odometry, pose_twist_callback)
    sub_scan = rospy.Subscriber('/scan', LaserScan, avoid_obs)
    global goal
    goal = Pose()
    pose = Pose()
    pose.position.x = round(pose.position.x, 4)
    pose.position.y = round(pose.position.y, 4)
    goal_pose = goal

    goal.position.x = float(input('Enter x coordinate:'))
    goal.position.y = float(input('Enter y coordinate:'))

    vel = Twist()

    rospy.wait_for_message('/odom', Odometry)

    orig_dis = sqrt(pow((goal.position.x - pose.position.x), 2) + pow((goal.position.y - pose.position.y), 2))

    rate = rospy.Rate(20)  # 50ms = 20Hz
    while dis > 0.1 and not rospy.is_shutdown():
        avoid_obstacle()
        rate.sleep()

    vel.linear.x = 0
    vel.angular.z = 0
    pub.publish(vel)
    rospy.spin()
