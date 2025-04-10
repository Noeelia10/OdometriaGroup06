#!/usr/bin/env python3

import rospy
from ejecutar_movimiento.srv import EjecutarMovimiento, EjecutarMovimientoResponse
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
import math

# Variables globales
yaw = 0.0
x_actual = None
Kp_rot = 2.0
Kp_lin = 2.0

def callback_odom(msg):
    """Actualiza la posición y orientación actual del robot."""
    global yaw, x_actual

    x_actual = msg.pose.pose.position.x
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (_, _, yaw) = euler_from_quaternion(orientation_list)

def manejar_peticion(req):
    """Maneja las solicitudes del cliente."""
    global yaw, x_actual

    rospy.loginfo(f"Solicitud recibida: Tipo -> {req.tipo_movimiento}, Valor -> {req.valor}")

    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    rospy.Subscriber('/odom', Odometry, callback_odom)
    rospy.sleep(1)

    if x_actual is None:
        rospy.logerr("No se han recibido datos de odometría.")
        return EjecutarMovimientoResponse(success=False)

    command = Twist()

    if req.tipo_movimiento == "lineal":
        # Asignar la posición inicial justo antes de iniciar el movimiento
        x_inicial = x_actual
        distancia_objetivo = req.valor
        distancia_recorrida = 0.0

        rospy.loginfo(f"Iniciando movimiento lineal desde {x_inicial:.2f} m")

        while abs(distancia_objetivo - distancia_recorrida) > 0.01 and not rospy.is_shutdown():
            distancia_recorrida = abs(x_actual - x_inicial)
            error = distancia_objetivo - distancia_recorrida

            # Control proporcional con limitación de velocidad
            vel = Kp_lin * error
            vel = max(min(vel, 0.2), -0.2)  # Limitar a ±0.2 m/s
            command.linear.x = vel
            command.angular.z = 0.0

            pub.publish(command)
            rospy.loginfo(f"Distancia: {distancia_recorrida:.2f} m, Error: {error:.2f} m")
            rospy.sleep(0.1)

        command.linear.x = 0.0
        pub.publish(command)
        rospy.loginfo("Movimiento lineal completado")
        return EjecutarMovimientoResponse(success=True)

    elif req.tipo_movimiento == "rotacion":
        incremento_rotacion = math.radians(req.valor)
        target_yaw = yaw + incremento_rotacion
        target_yaw = math.atan2(math.sin(target_yaw), math.cos(target_yaw))  # Normalizar

        rospy.loginfo(f"Rotando {req.valor} grados (objetivo: {math.degrees(target_yaw):.2f}°)")

        while not rospy.is_shutdown():
            error = target_yaw - yaw
            error = math.atan2(math.sin(error), math.cos(error))  # Mantener entre [-π, π]

            if abs(error) < 0.02:
                command.angular.z = 0.0
                pub.publish(command)
                rospy.loginfo(f"Rotación completada: {math.degrees(yaw):.2f}°")
                return EjecutarMovimientoResponse(success=True)

            vel = Kp_rot * error
            vel = max(min(vel, 1.0), -1.0)  # Limitar a ±1.0 rad/s
            command.linear.x = 0.0
            command.angular.z = vel
            pub.publish(command)
            rospy.sleep(0.1)

    return EjecutarMovimientoResponse(success=False)

def iniciar_servidor():
    rospy.init_node('servidor_movimiento')
    servicio = rospy.Service('ejecutar_movimiento', EjecutarMovimiento, manejar_peticion)
    rospy.loginfo("Servidor de movimiento listo y esperando solicitudes...")
    rospy.spin()

if __name__ == "__main__":
    iniciar_servidor()

