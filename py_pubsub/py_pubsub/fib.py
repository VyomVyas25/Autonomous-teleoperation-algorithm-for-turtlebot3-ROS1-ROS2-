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

    curr_vel=0.01*math.pi/2
    curr_ang=math.pi*2
    k=0
    i=0
    j=1

    while k<=89 and  rclpy.ok():
        vel.linear.x = curr_vel
        vel.angular.z = curr_ang      
        pub.publish(vel)  
        k=i+j
        j=i
        i=k
        print(k)
        curr_vel+=0.1*k
        curr_ang-=0.002*k
        time.sleep(0.5)
    vel.linear.x=0.0
    vel.angular.z=0.0
    pub.publish(vel)
    
if __name__ == '__main__':
    main()