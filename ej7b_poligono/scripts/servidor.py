#!/usr/bin/env python3
import rospy
from ej7b_poligono.srv import DibujarPoligono, DibujarPoligonoResponse
from geometry_msgs.msg import Twist
import math

def manejar_solicitud(req):
    rospy.loginfo(f"Solicitud recibida: {req.n_lados} lados, {req.longitud} metros.")

    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    rate = rospy.Rate(10)
    command = Twist()

    if req.n_lados > 10:
        # Dibujar un círculo si n_lados > 10
        radio = req.longitud / 2.0  # Convertimos el diámetro a radio
        command.linear.x = 0.2  # Velocidad lineal fija
        command.angular.z = command.linear.x / radio  # Relación de velocidad angular
        
        tiempo_mov = 2 * math.pi * radio / command.linear.x  # Tiempo para completar el círculo
        tiempo_final = rospy.Time.now() + rospy.Duration(tiempo_mov)

        while rospy.Time.now() < tiempo_final:
            pub.publish(command)
            rate.sleep()
        
        command.linear.x = 0
        command.angular.z = 0
        pub.publish(command)
    
    else:
        # Dibujar polígono normal
        angulo_giro = 360.0 / req.n_lados  # Ángulo en grados
        angulo_giro_rad = math.radians(angulo_giro)  # Convertir a radianes
        
        for _ in range(req.n_lados):
            # Mover en línea recta
            command.linear.x = 0.1  # Velocidad fija
            for _ in range(int(100 * req.longitud)):  # Aproximadamente longitud metros
                pub.publish(command)
                rate.sleep()
            
            command.linear.x = 0
            pub.publish(command)
            rospy.sleep(1)  # Pequeña pausa antes del giro
            
            # Girar el ángulo correcto
            command.angular.z = angulo_giro_rad / 2  # Ajuste fino
            tiempo_giro = angulo_giro_rad / command.angular.z
            tiempo_final = rospy.Time.now() + rospy.Duration(tiempo_giro)

            while rospy.Time.now() < tiempo_final:
                pub.publish(command)
                rate.sleep()

            command.angular.z = 0
            pub.publish(command)
            rospy.sleep(1)  # Pequeña pausa antes del siguiente lado
    
    return DibujarPoligonoResponse(exito=True)

def iniciar_servidor():
    rospy.init_node('servidor_poligono')
    servicio = rospy.Service('dibujar_figura', DibujarPoligono, manejar_solicitud)
    rospy.loginfo("Servidor de polígonos esperando solicitudes...")
    rospy.spin()

if __name__ == "__main__":
    iniciar_servidor()
