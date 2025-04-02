#!/usr/bin/env python3
import rospy
from ejecutar_movimiento.srv import EjecutarMovimiento, EjecutarMovimientoResponse
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
import math

yaw = 0.0
x_inicial = None
x_actual = None
Kp_rot = 2.0  # Aumentamos la ganancia para que reaccione mejor en giros grandes
Kp_lin = 2.0  

def callback_odom(msg):
    """Función que actualiza la posición y orientación del robot"""
    global x_inicial, x_actual, yaw
    x_actual = msg.pose.pose.position.x
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    _, _, yaw = euler_from_quaternion(orientation_list)  # Convertimos a Euler
    if x_inicial is None:
        x_inicial = x_actual

def manejar_peticion(req):
    """Función que maneja las solicitudes del cliente"""
    global x_inicial, x_actual, yaw

    rospy.loginfo(f"Solicitud recibida: Tipo -> {req.tipo_movimiento}, Valor -> {req.valor}")

    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    rospy.Subscriber('/odom', Odometry, callback_odom)
    rospy.sleep(1)  # Esperar para recibir datos de /odom
    rate = rospy.Rate(10)
    command = Twist()

    if x_actual is None:
        rospy.logerr("No se han recibido datos de odometría. Asegúrate de que /odom está publicando.")
        return EjecutarMovimientoResponse(success=False)

    if req.tipo_movimiento == "lineal":
        x_inicial = x_actual
        distancia_objetivo = req.valor
        rospy.loginfo(f"Moviendo en línea recta {distancia_objetivo} metros.")

        while not rospy.is_shutdown():
            distancia_recorrida = abs(x_actual - x_inicial)
            error = distancia_objetivo - distancia_recorrida

            if abs(error) < 0.02:
                command.linear.x = 0
                pub.publish(command)
                rospy.loginfo("Movimiento lineal completado")
                return EjecutarMovimientoResponse(success=True)

            command.linear.x = Kp_lin * error
            pub.publish(command)
            rate.sleep()

    elif req.tipo_movimiento == "rotacion":
        target_rad = math.radians(req.valor)
        rospy.loginfo(f"Iniciando rotación de {req.valor} grados ({target_rad:.2f} radianes)")

        while not rospy.is_shutdown():
            error = target_rad - yaw

            # Asegurar que el error de rotación esté en el rango [-π, π]
            error = math.atan2(math.sin(error), math.cos(error))

            # Si el error es pequeño, detenemos el giro
            if abs(error) < 0.02:
                command.angular.z = 0
                pub.publish(command)
                rospy.loginfo(f"Rotación completada: {math.degrees(yaw):.2f} grados")
                return EjecutarMovimientoResponse(success=True)

            command.angular.z = Kp_rot * error  # Ajustamos la ganancia proporcional
            pub.publish(command)
            rate.sleep()

    return EjecutarMovimientoResponse(success=False)

def iniciar_servidor():
    """Inicia el servicio"""
    rospy.init_node('servidor_movimiento')
    servicio = rospy.Service('ejecutar_movimiento', EjecutarMovimiento, manejar_peticion)
    rospy.loginfo("Servidor de movimiento listo y esperando solicitudes...")
    rospy.spin()

if __name__ == "__main__":
    iniciar_servidor()
