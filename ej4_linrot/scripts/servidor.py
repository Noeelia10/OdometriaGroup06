#!/usr/bin/env python3
import rospy
from seleccion_movimiento.srv import SeleccionMovimiento, SeleccionMovimientoResponse

def manejar_peticion(req):
    rospy.loginfo("Solicitud recibida: Tipo de movimiento -> {}, Valor -> {}".format(req.tipo_movimiento, req.valor))
    return SeleccionMovimientoResponse(True)  # Confirmamos la recepción

if __name__ == '__main__':
    rospy.init_node('servidor_movimiento')
    servicio = rospy.Service('seleccion_movimiento', SeleccionMovimiento, manejar_peticion)
    rospy.loginfo("Servidor esperando solicitudes...")
    rospy.spin()
