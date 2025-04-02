#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math

# Variables globales para la posición inicial y actual
x_inicial = None
x_actual = None

# Constante de control proporcional
Kp = 0.2  
distancia_objetivo = 1.0  # Queremos movernos 1 metro

# Función para obtener la posición actual del robot
def callback_odom(msg):
    global x_inicial, x_actual
    x_actual = msg.pose.pose.position.x

    # Guardamos la posición inicial la primera vez
    if x_inicial is None:
        x_inicial = x_actual

if __name__ == '__main__':
    rospy.init_node('mover_1m_lazo')
    
    # Suscribirse al tópico /odom para recibir la odometría
    rospy.Subscriber('/odom', Odometry, callback_odom)
    
    # Publicador de velocidad en /cmd_vel
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    
    rate = rospy.Rate(10)
    command = Twist()

    # Esperamos a que se reciba la primera lectura de odometría
    rospy.loginfo("Esperando odometría...")
    while x_actual is None:
        rospy.sleep(0.1)

    rospy.loginfo("Posición inicial registrada: {:.2f}".format(x_inicial))

    while not rospy.is_shutdown():
        # Calculamos la distancia recorrida
        distancia_recorrida = abs(x_actual - x_inicial)
        error = distancia_objetivo - distancia_recorrida

        # Si el error es pequeño, nos detenemos
        if abs(error) < 0.01:
            command.linear.x = 0
            pub.publish(command)
            rospy.loginfo("Meta alcanzada: 1 metro recorrido")
            break

        # Control proporcional de velocidad
        command.linear.x = Kp * error
        pub.publish(command)

        rospy.loginfo("Distancia recorrida: {:.2f}, Error: {:.2f}".format(distancia_recorrida, error))
        rate.sleep()
