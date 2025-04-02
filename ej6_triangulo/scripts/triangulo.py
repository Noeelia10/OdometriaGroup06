#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import math

def mover_recto(pub, distancia, velocidad=0.2):
    """Mueve el robot en línea recta una distancia dada en metros"""
    rate = rospy.Rate(10)
    command = Twist()
    command.linear.x = velocidad
    tiempo_movimiento = distancia / velocidad  # Tiempo estimado para recorrer la distancia

    rospy.loginfo(f"Moviendo {distancia} metros en línea recta...")
    tiempo_inicio = rospy.Time.now().to_sec()

    while rospy.Time.now().to_sec() - tiempo_inicio < tiempo_movimiento:
        pub.publish(command)
        rate.sleep()

    command.linear.x = 0
    pub.publish(command)
    rospy.sleep(1)

def girar(pub, angulo_grados, velocidad_angular=0.5):
    """Gira el robot el número de grados especificado"""
    rate = rospy.Rate(10)
    command = Twist()
    command.angular.z = velocidad_angular
    tiempo_giro = math.radians(angulo_grados) / velocidad_angular  # Tiempo estimado para girar

    rospy.loginfo(f"Rotando {angulo_grados} grados...")
    tiempo_inicio = rospy.Time.now().to_sec()

    while rospy.Time.now().to_sec() - tiempo_inicio < tiempo_giro:
        pub.publish(command)
        rate.sleep()

    command.angular.z = 0
    pub.publish(command)
    rospy.sleep(1)

def recorrer_triangulo(lado):
    """Ejecuta el movimiento para formar un triángulo equilátero"""
    rospy.init_node('triangulo_robot', anonymous=True)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    rospy.sleep(2)  # Esperar a que el nodo y el publisher se inicialicen

    for _ in range(3):
        mover_recto(pub, lado)
        girar(pub, 120)

    rospy.loginfo("Triángulo completado.")

if __name__ == "__main__":
    try:
        lado = float(input("Introduce la longitud del lado del triángulo en metros: "))
        recorrer_triangulo(lado)
    except rospy.ROSInterruptException:
        pass
