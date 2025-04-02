#!/usr/bin/env python3

import rospy
from ejecutar_movimiento.srv import EjecutarMovimiento, EjecutarMovimientoResponse
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
import math

# Variables globales
yaw = 0.0  # Ángulo actual del robot
x_actual = None
x_inicial = None
Kp_rot = 2.0  # Ganancia para el control proporcional de rotación
Kp_lin = 2.0  # Ganancia para el control proporcional de traslación

def callback_odom(msg):
    """Función que actualiza la posición y orientación del robot."""
    global yaw, x_actual, x_inicial

    x_actual = msg.pose.pose.position.x
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (_, _, yaw) = euler_from_quaternion(orientation_list)  # Convertimos a Euler

    # Guardamos la posición inicial la primera vez que se recibe odometría
    if x_inicial is None:
        x_inicial = x_actual

def manejar_peticion(req):
    """Función que maneja las solicitudes del cliente."""
    global yaw, x_actual, x_inicial

    rospy.loginfo(f"Solicitud recibida: Tipo -> {req.tipo_movimiento}, Valor -> {req.valor}")

    # Publicador de velocidad
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    rospy.Subscriber('/odom', Odometry, callback_odom)

    # Esperamos a que se reciban datos de odometría
    rospy.sleep(1)

    if x_actual is None:
        rospy.logerr("No se han recibido datos de odometría. Asegúrate de que /odom está publicando.")
        return EjecutarMovimientoResponse(success=False)

    command = Twist()

    if req.tipo_movimiento == "lineal":
        # Movimiento en línea recta con control proporcional
        distancia_objetivo = req.valor
        distancia_recorrida = 0

        while abs(distancia_objetivo - distancia_recorrida) > 0.02:
            distancia_recorrida = abs(x_actual - x_inicial)
            error = distancia_objetivo - distancia_recorrida

            command.linear.x = Kp_lin * error
            pub.publish(command)
            rospy.sleep(0.1)

        # Detener el movimiento
        command.linear.x = 0
        pub.publish(command)
        rospy.loginfo("Movimiento lineal completado")
        return EjecutarMovimientoResponse(success=True)

    elif req.tipo_movimiento == "rotacion":
        # Convertimos el ángulo de grados a radianes
        incremento_rotacion = math.radians(req.valor)  
        target_yaw = yaw + incremento_rotacion  # Ángulo objetivo es relativo al actual

        rospy.loginfo(f"Rotando {req.valor} grados desde la posición actual")

        while not rospy.is_shutdown():
            error = target_yaw - yaw

            # Asegurar que el error está dentro del rango correcto [-π, π]
            error = math.atan2(math.sin(error), math.cos(error))

            if abs(error) < 0.02:  # Si el error es pequeño, detener el giro
                command.angular.z = 0
                pub.publish(command)
                rospy.loginfo(f"Rotación completada: {math.degrees(yaw):.2f} grados")
                return EjecutarMovimientoResponse(success=True)

            command.angular.z = Kp_rot * error  # Control proporcional
            pub.publish(command)
            rospy.sleep(0.1)

    return EjecutarMovimientoResponse(success=False)

def iniciar_servidor():
    """Inicia el servicio."""
    rospy.init_node('servidor_movimiento')
    servicio = rospy.Service('ejecutar_movimiento', EjecutarMovimiento, manejar_peticion)
    rospy.loginfo("Servidor de movimiento listo y esperando solicitudes...")
    rospy.spin()

if __name__ == "__main__":
    iniciar_servidor()
