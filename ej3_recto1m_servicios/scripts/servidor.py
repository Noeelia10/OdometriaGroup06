#!/usr/bin/env python3
import rospy
from servicio_movimiento.srv import MoverRobot, MoverRobotResponse
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math

x_inicial = None
x_actual = None
Kp = 0.2

def callback_odom(msg):
    global x_inicial, x_actual
    x_actual = msg.pose.pose.position.x
    if x_inicial is None:
        x_inicial = x_actual

def manejar_peticion(req):
    global x_inicial, x_actual
    rospy.loginfo("Solicitud recibida para mover {} metros".format(req.distancia))

    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    rospy.Subscriber('/odom', Odometry, callback_odom)
    rate = rospy.Rate(10)
    command = Twist()

    while x_actual is None:
        rospy.sleep(0.1)

    x_inicial = x_actual
    distancia_objetivo = req.distancia

    while not rospy.is_shutdown():
        distancia_recorrida = abs(x_actual - x_inicial)
        error = distancia_objetivo - distancia_recorrida

        if abs(error) < 0.01:
            command.linear.x = 0
            pub.publish(command)
            rospy.loginfo("Meta alcanzada: {} metros".format(req.distancia))
            return MoverRobotResponse(True)

        command.linear.x = Kp * error
        pub.publish(command)
        rate.sleep()

if __name__ == '__main__':
    rospy.init_node('servidor_movimiento')
    servicio = rospy.Service('mover_robot', MoverRobot, manejar_peticion)
    rospy.loginfo("Servidor listo para recibir solicitudes")
    rospy.spin()
