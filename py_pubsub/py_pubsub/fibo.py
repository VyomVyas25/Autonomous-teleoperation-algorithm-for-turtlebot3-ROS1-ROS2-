import rclpy
from geometry_msgs.msg import Twist
import math
import time
   
def main(args=None):
    global vel, pub, sub, node

    rclpy.init(args=args)
    node = rclpy.create_node('fibonacci_spiral')
    pub = node.create_publisher(Twist, 'turtle1/cmd_vel', 10)
    vel=Twist()

    curr_vel=0.3
    curr_ang=math.pi/3
    spiral=0.01
    j=5

    while rclpy.ok():
        vel.linear.x = curr_vel
        vel.angular.z = curr_ang      
        pub.publish(vel)  
        curr_vel+=spiral
        time.sleep(0.1)
    
if __name__ == '__main__':
    main()
    
