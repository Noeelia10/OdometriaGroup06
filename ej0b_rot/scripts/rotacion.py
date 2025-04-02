#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist
import math

roll = pitch = yaw = 0.0
target = None  
kp = 0.5  # Ganancia del control proporcional

def get_rotation(msg: Odometry):
    global roll, pitch, yaw
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)

if __name__ == '__main__':
    rospy.init_node('rotate_robot')
    sub = rospy.Subscriber('/odom', Odometry, get_rotation)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    r = rospy.Rate(10)
    command = Twist()
    rospy.wait_for_message('/odom', Odometry)
    target = yaw + math.radians(90)

    while not rospy.is_shutdown():
        
    	command.angular.z = kp * (target - yaw)  # Control proporcional
    	pub.publish(command)
    	rospy.loginfo(f'Targeta: {target}, Current: {yaw}')
    	r.sleep()

