#!/usr/bin/env python3
from re import S
import rospy
import math
from rospy.core import is_shutdown
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point
from tf.transformations import euler_from_quaternion


def move():

    run = Twist()

    run.linear.x = 0.5
    pub.publish(run)

    



def odom_callback(message):

    speed = Twist()

    current_x = message.pose.pose.position.x
    current_y = message.pose.pose.position.y

    rot_q = message.pose.pose.orientation

    (R, P, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
    
    goal = (-2,-5)
    
    goalx, goaly = goal

    incx = goalx-current_x
    incy = goaly-current_y

    angle_to_rotate = math.atan2(incy,incx)

    if(abs(angle_to_rotate-theta) > 0.1):
        speed.linear.x = 0
        speed.angular.z = 0.5

    else:
        speed.linear.x = 0.5
        speed.angular.z = 0.0

    if abs(incx < 0.3) and abs(incy<0.3):
        speed.linear.x = 0
        speed.angular.z = 0.0

    pub.publish(speed)



    


if __name__ == '__main__':

    rospy.init_node('astar')
    rate = rospy.Rate(5)
    odom_sub = rospy.Subscriber('/base_pose_ground_truth', Odometry, odom_callback)
    pub = rospy.Publisher('/cmd_vel' , Twist, queue_size= 1)

    
    


   

    
    rospy.spin()
