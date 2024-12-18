import rclpy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import sqrt, pow, atan2

def pose_callback(curr_pose):
    global x_self, y_self, theta_self, dis
    x_self = curr_pose.x
    y_self = curr_pose.y
    theta_self = curr_pose.theta
    dis = sqrt(pow(x_goal - x_self, 2) + pow(y_goal - y_self, 2))

def g2g():
    global x_goal, y_goal
    x_goal = float(input("Enter goal x: "))
    y_goal = float(input("Enter goal y: "))

def main(args=None):
    global x_goal, y_goal, x_self, y_self, theta_self, vel, pub, sub, dis
    rclpy.init(args=args)
    node = rclpy.create_node('ttlsim')

    pub = node.create_publisher(Twist, 'turtle1/cmd_vel', 10)
    sub = node.create_subscription(Pose, 'turtle1/pose', pose_callback, 10)
    
    vel = Twist()
    while rclpy.ok():
        g2g() 
        dis = float('inf')
        while rclpy.ok() and dis > 0.01:
            rclpy.spin_once(node, timeout_sec=0.1) 
            angle_diff = atan2((y_goal - y_self), (x_goal - x_self)) - theta_self
            vel.linear.x = 0.75 * dis
            vel.angular.z = 3 * angle_diff
            pub.publish(vel)
        
if __name__ == '__main__':
    main()