#!/usr/bin/env python3

import rospy
import moveit_commander
from geometry_msgs.msg import Pose, PoseStamped
from moveit_commander import PlanningSceneInterface

def move_to_pose(group, x, y, z):
    group.set_start_state_to_current_state()
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    pose.orientation.x = 0.0
    pose.orientation.y = 1.0  # orientación desde arriba
    pose.orientation.z = 0.0
    pose.orientation.w = 0.0
    group.set_pose_target(pose)
    group.go(wait=True)
    group.stop()
    group.clear_pose_targets()

def control_gripper(gripper, open=True):
    # Si open = true, target = [0, 0] -> abre pinza, si no, la cierra
    target = [0.0, 0.0] if open else [0.4, 0.4]
    
    # set_joint_value_target hace que los dedos del gripper se muevan a la posición target
    gripper.set_joint_value_target(target)

    # Comando que ejecuta el movimiento a la espera del fin de la ejecución, cuando wait = True
    success = gripper.go(wait=True)

    gripper.stop()
    rospy.sleep(1)

    if not success:
        rospy.logwarn("Movimiento del gripper no se completó correctamente.")


def add_cylinder(scene, frame_id):
    rospy.sleep(1)
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.pose.position.x = 0.4
    pose.pose.position.y = 0.0
    pose.pose.position.z = 0.06  # centro del cilindro
    pose.pose.orientation.w = 1.0
    scene.add_cylinder("lata", pose, height=0.12, radius=0.035)
    rospy.loginfo("Lata añadida a la escena")
    rospy.sleep(2)

def remove_cylinder(scene):
    scene.remove_world_object("lata")
    rospy.loginfo("Lata eliminada de la escena")
    rospy.sleep(1)

def main():
    moveit_commander.roscpp_initialize([])
    rospy.init_node('trabajo_kinova_node', anonymous=True)

    arm = moveit_commander.MoveGroupCommander("arm")
    gripper = moveit_commander.MoveGroupCommander("gripper")
    scene = PlanningSceneInterface()
    rospy.sleep(1)

    # Obtener el frame correcto automáticamente
    planning_frame = arm.get_planning_frame()
    rospy.loginfo(f"Usando frame de planificación: {planning_frame}")
    add_cylinder(scene, frame_id=planning_frame)

    # 1) Mover a P1 (30cm sobre la lata)
    rospy.loginfo("1. Ir a P1 (encima de la lata)")
    move_to_pose(arm, 0.4, 0.0, 0.3)

    # 2) Bajar a PA (posición de agarre)
    rospy.loginfo("2. Bajar a PA (agarre)")
    move_to_pose(arm, 0.4, 0.0, 0.12)

    # 3) Cerrar pinza
    rospy.loginfo("3. Cerrar pinza")
    control_gripper(gripper, open=False)

    # 4) Subir de nuevo a P1
    rospy.loginfo("4. Subir a P1")
    move_to_pose(arm, 0.4, 0.0, 0.3)

    # 5) Ir a P2 (encima del destino)
    rospy.loginfo("5. Ir a P2 (encima del destino)")
    move_to_pose(arm, 0.0, 0.4, 0.3)

    # 6) Bajar a PS (posición de soltar)
    rospy.loginfo("6. Bajar a PS (soltar)")
    move_to_pose(arm, 0.0, 0.4, 0.12)

    # 7) Abrir pinza
    rospy.loginfo("7. Abrir pinza")
    control_gripper(gripper, open=True)

    # 8) Subir a P2 otra vez
    rospy.loginfo("8. Subir a P2")
    move_to_pose(arm, 0.0, 0.4, 0.3)

    # Eliminar la lata de la escena
    remove_cylinder(scene)

    # 9) Volver a posición Home
    rospy.loginfo("9. Volver a posición Home")
    arm.set_named_target("Home")
    arm.go(wait=True)
    arm.stop()

    rospy.loginfo("Trabajo completado con éxito")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
