#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math

class NoCollisionStopper:
    def __init__(self):
        rospy.init_node('nodo_nocolision_simple')

        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/scan', LaserScan, self.lidar_callback)

        self.distancia_segura = 1.0  # metros
        self.angulo_seguridad = math.radians(60)  # ±60°

        self.obstaculo_detectado = False

        # Publicar continuamente si hay obstáculo
        rospy.Timer(rospy.Duration(0.1), self.enviar_parada_si_necesaria)

        rospy.loginfo("Nodo de frenado por colisión iniciado.")

    def lidar_callback(self, scan_msg):
        ranges = scan_msg.ranges
        angle_increment = scan_msg.angle_increment
        total_rays = len(ranges)

        if total_rays == 0:
            self.obstaculo_detectado = False
            return

        center_index = total_rays // 2
        index_offset = int(self.angulo_seguridad / angle_increment)

        start = max(center_index - index_offset, 0)
        end = min(center_index + index_offset, total_rays)

        frontal = [r for r in ranges[start:end] if 0.0 < r < float('inf')]

        if frontal and min(frontal) < self.distancia_segura:
            self.obstaculo_detectado = True
            rospy.logwarn("Obstáculo a %.2f m: deteniendo el robot." % min(frontal))
        else:
            self.obstaculo_detectado = False

    def enviar_parada_si_necesaria(self, event):
        if self.obstaculo_detectado:
            stop_msg = Twist()
            self.cmd_pub.publish(stop_msg)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    nodo = NoCollisionStopper()
    nodo.run()
