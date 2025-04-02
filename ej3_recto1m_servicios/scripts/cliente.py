#!/usr/bin/env python3
import rospy
from servicio_movimiento.srv import MoverRobot, MoverRobotRequest

rospy.init_node('cliente_movimiento')
rospy.wait_for_service('mover_robot')

try:
    mover_robot = rospy.ServiceProxy('mover_robot', MoverRobot)
    distancia = 1.0  # Pedimos mover 1 metro
    respuesta = mover_robot(distancia)
    if respuesta.exito:
        rospy.loginfo("El robot se movió exitosamente {} metros".format(distancia))
    else:
        rospy.loginfo("Hubo un problema en el movimiento")
except rospy.ServiceException as e:
    rospy.logerr("Error en el servicio: {}".format(e))
