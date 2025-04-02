#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math

yaw = 0.0  # Variable global para almacenar el ángulo actual

def get_rotation(msg):
    """ Callback para obtener el ángulo de rotación actual del robot """
    global yaw
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (_, _, yaw) = euler_from_quaternion(orientation_list)  # Obtener ángulo en radianes

def mover_poligono(n_lados, longitud):
    """ Función para mover el robot en forma de polígono """
    global yaw

    rospy.init_node('movimiento_poligono', anonymous=True)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    sub = rospy.Subscriber('/odom', Odometry, get_rotation)  # Escuchar la odometría

    rate = rospy.Rate(10)
    command = Twist()

    angulo_giro = 360 / n_lados  # Ángulo en grados
    angulo_giro_rad = math.radians(angulo_giro)  # Convertir a radianes

    for _ in range(n_lados):
        # Mover en línea recta
        command.linear.x = 0.1  # Velocidad
        for _ in range(int(100 * longitud)):  # Aproximadamente L metros
            pub.publish(command)
            rate.sleep()

        command.linear.x = 0  # Detener el movimiento lineal
        pub.publish(command)
        rospy.sleep(1)  # Pequeña pausa antes del giro

        # Girar usando odometría en lugar de temporización
        angulo_inicial = yaw
        angulo_deseado = angulo_inicial + angulo_giro_rad  # Ángulo objetivo

        # Normalizar el ángulo deseado entre -pi y pi
        angulo_deseado = (angulo_deseado + math.pi) % (2 * math.pi) - math.pi  

        command.angular.z = 0.3  # Velocidad de giro

        while abs(yaw - angulo_deseado) > 0.02:  # Permitir un pequeño error
            pub.publish(command)
            rate.sleep()

        command.angular.z = 0  # Detener el giro
        pub.publish(command)
        rospy.sleep(1)  # Pequeña pausa antes del siguiente lado

    rospy.loginfo("Polígono completado correctamente")

if __name__ == '__main__':
    try:
        n_lados = int(input("Ingrese el número de lados: "))
        longitud = float(input("Ingrese la longitud de lado deseada: "))
        mover_poligono(n_lados, longitud)
    except rospy.ROSInterruptException:
        pass
