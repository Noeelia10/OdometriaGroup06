#!/usr/bin/env python3
import rospy
from ejecutar_movimiento.srv import EjecutarMovimiento

def solicitar_movimiento():
    rospy.wait_for_service('ejecutar_movimiento')
    try:
        ejecutar_movimiento = rospy.ServiceProxy('ejecutar_movimiento', EjecutarMovimiento)

        tipo = input("¿Qué tipo de movimiento deseas? (lineal/rotacion): ").strip().lower()
        if tipo not in ["lineal", "rotacion"]:
            rospy.logerr("Movimiento inválido. Debe ser 'lineal' o 'rotacion'.")
            return

        valor = float(input("Introduce el valor (metros para lineal, grados para rotación): "))

        respuesta = ejecutar_movimiento(tipo, valor)
        if respuesta.success:
            rospy.loginfo("Movimiento ejecutado correctamente.")
        else:
            rospy.logwarn("Hubo un problema al ejecutar el movimiento.")

    except rospy.ServiceException as e:
        rospy.logerr(f"Error en la llamada al servicio: {e}")

if __name__ == "__main__":
    rospy.init_node('cliente_movimiento')
    solicitar_movimiento()
