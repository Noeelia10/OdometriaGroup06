#!/usr/bin/env python3
import rospy
from seleccion_movimiento.srv import SeleccionMovimiento, SeleccionMovimientoRequest

rospy.init_node('cliente_movimiento')
rospy.wait_for_service('seleccion_movimiento')

try:
    seleccion_movimiento = rospy.ServiceProxy('seleccion_movimiento', SeleccionMovimiento)

    tipo_mov = input("¿Qué tipo de movimiento deseas? (lineal/rotacion): ").strip().lower()
    if tipo_mov not in ["lineal", "rotacion"]:
        rospy.logerr("Movimiento inválido. Debe ser 'lineal' o 'rotacion'.")
        exit(1)

    valor = float(input("Introduce el valor (metros para lineal, grados para rotación): "))

    respuesta = seleccion_movimiento(tipo_mov, valor)
    if respuesta.confirmacion:
        rospy.loginfo("Movimiento enviado correctamente.")
    else:
        rospy.loginfo("Error en el envío.")
except rospy.ServiceException as e:
    rospy.logerr("Error en el servicio: {}".format(e))
