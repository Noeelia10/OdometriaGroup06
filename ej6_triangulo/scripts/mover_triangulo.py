#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import math

def mover_triangulo(longitud_lado):
    rospy.init_node('mover_triangulo')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    rate = rospy.Rate(10)
    command = Twist()

    # Número de lados del triángulo
    for _ in range(3):
        # Movimiento recto (lado del triángulo)
        command.linear.x = 0.05  # Velocidad lineal constante
        command.angular.z = 0.0
        tiempo_mov = longitud_lado / command.linear.x  # Tiempo para recorrer L metros
        tiempo_final = rospy.Time.now() + rospy.Duration(tiempo_mov)

        while rospy.Time.now() < tiempo_final:
            pub.publish(command)
            rate.sleep()
        
        # Detener el robot
        command.linear.x = 0.0
        pub.publish(command)
        rospy.sleep(1)  # Pequeña pausa

        # Rotación de 120° en radianes
        command.angular.z = math.radians(120) / 2  # Ajuste experimental de velocidad
        tiempo_rot = 2  # Aproximado, ajustar si es necesario
        tiempo_final = rospy.Time.now() + rospy.Duration(tiempo_rot)

        while rospy.Time.now() < tiempo_final:
            pub.publish(command)
            rate.sleep()

        # Detener el robot después de la rotación
        command.angular.z = 0.0
        pub.publish(command)
        rospy.sleep(1)

if __name__ == "__main__":
    try:
        longitud_lado = float(input("Introduce la longitud del lado del triángulo (en metros): "))
        mover_triangulo(longitud_lado)
    except rospy.ROSInterruptException:
        pass
