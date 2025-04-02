#!/usr/bin/env python3
import rospy
from ej7b_poligono.srv import DibujarPoligono, DibujarPoligonoRequest

def solicitar_poligono():
    rospy.init_node('cliente_poligono')  # Inicializar el nodo
    rospy.loginfo("Esperando a que el servicio esté disponible...")

    rospy.wait_for_service('dibujar_figura')  # Esperar a que el servicio esté activo
    rospy.loginfo("Conectado al servicio, enviando solicitud...")

    try:
        # Crear el proxy para llamar al servicio
        dibujar_figura = rospy.ServiceProxy('dibujar_figura', DibujarPoligono)

        # Solicitar datos al usuario
        n_lados = int(input("Número de lados del polígono (más de 10 será un círculo): "))
        longitud = float(input("Longitud de los lados (o diámetro si es un círculo): "))

        # Crear la solicitud
        solicitud = DibujarPoligonoRequest()
        solicitud.n_lados = n_lados
        solicitud.longitud = longitud

        # Enviar la solicitud y recibir la respuesta
        respuesta = dibujar_figura(solicitud)

        # Evaluar la respuesta del servidor
        if respuesta.exito:
            rospy.loginfo(" El polígono fue dibujado correctamente.")
        else:
            rospy.logwarn("️ Hubo un problema al dibujar el polígono.")

    except rospy.ServiceException as e:
        rospy.logerr(f" Error al llamar al servicio: {e}")

if __name__ == "__main__":
    solicitar_poligono()
