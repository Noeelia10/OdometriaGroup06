#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import time

def mover_recto():
	rospy.init_node('mover_recto', anonymous=True)
	pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
	rate = rospy.Rate(10) # 10 Hz
	cmd = Twist()

	# Configurar velocidad
	cmd.linear.x = -0.05 # Velocidad de 0.05 m/s
	cmd.angular.z = 0.0 # Sin giro

	rospy.loginfo("Movimiento hacia adelante...")

	start_time = time.time()
	while time.time() - start_time < 20: # Mueve durante 20 segundos (1 metro)
		pub.publish(cmd)
		rate.sleep()

	rospy.loginfo("Deteniendo el robot")
	cmd.linear.x = 0.0
	pub.publish(cmd)

if __name__ == '__main__':
	try:
		mover_recto()
	except rospy.ROSInterruptException:
		pass
