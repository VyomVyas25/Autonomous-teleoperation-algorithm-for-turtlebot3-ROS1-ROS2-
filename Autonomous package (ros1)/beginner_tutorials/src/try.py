#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from math import sqrt, atan2, sin, cos, pi
from tf.transformations import euler_from_quaternion

global dis ,an_diff, angle_diff, angle, current_pitch, current_roll, current_yaw
y = []

def pose_twist_callback(odom_msg):
    global dis, an_diff, angle_diff, angle, current_pitch, current_roll, current_yaw
    pose = odom_msg.pose.pose
    pose.position.x = round(pose.position.x, 4)
    pose.position.y = round(pose.position.y, 4)
    goal_pose = goal

    dis = sqrt(pow((goal_pose.position.x - pose.position.x), 2) + pow((goal_pose.position.y - pose.position.y), 2))
    current_roll, current_pitch, current_yaw = euler_from_quaternion([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
    angle = atan2(goal_pose.position.y - pose.position.y, goal_pose.position.x - pose.position.x)
    angle_diff = atan2(sin(angle - current_yaw), cos(angle - current_yaw))
    
    if angle_diff > (3 * pi / 4):
        an_diff = -(angle_diff)
    else:
        an_diff = angle_diff

def avoid_obs(msg):
    global y
    y = [v if v != 0 and v <= 1.65 else 1.65 for v in msg.ranges]

def avoid_obstacle():
    global dis_1
    val_max_1 = max(y[0:61])
    val_max_2 = max(y[300:360])
    val_min_i = y.index(max(val_max_1, val_max_2)) 

    try:  
        x1 = (val_max_1 - val_max_2) / abs(val_max_1 - val_max_2)
    except ZeroDivisionError:
        x1 = -1

    val_max_a = (val_min_i + (x1 * 2.5)) % len(y)
    dis_1 = max(v < 1.65 for v in y[300:] + y[:61])
    angle_diff_1 = atan2(sin((val_min_i - val_max_a)), cos(val_max_a - val_min_i))

    try:
        m = abs(dis_1) / abs(dis_1)
    except ZeroDivisionError:
        m = 0.0

    if dis_1 < 0.9 and dis_1 > 0.3:
        n = -1
    else:
        n = 1
    
    k_goal = 0.14 - (0.0145 * dis) + (0.0005 * (dis ** 2))
    k_obs = 1.6 + 0.225 * dis_1 - (0.6 * (dis_1 ** 2))
    v1 = n * (k_goal * (dis - (m * (dis_1 - 0.9))))
    v2 = (1 - (m * k_obs)) * an_diff + m * (k_obs * angle_diff_1)

    vel.linear.x = v1
    vel.angular.z = v2 * 0.5
    pub.publish(vel)

if __name__ == '__main__':
    rospy.init_node('fin_g2g', anonymous=True)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    sub_odom = rospy.Subscriber('/odom', Odometry, pose_twist_callback)
    sub_scan = rospy.Subscriber('/scan', LaserScan, avoid_obs)
    goal = Pose()

    goal.position.x = float(input('Enter x coordinate:'))
    goal.position.y = float(input('Enter y coordinate:'))
    vel = Twist()

    rospy.wait_for_message('/odom', Odometry)

    while dis > 0.1 and not rospy.is_shutdown():
        avoid_obstacle()
        rospy.sleep(0.05)

    vel.linear.x = 0
    vel.angular.z = 0
    pub.publish(vel)
    rospy.spin()
