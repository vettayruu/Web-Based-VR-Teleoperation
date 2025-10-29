from MQTT_Client import MQTT_Client
import sys
import time
import numpy as np
from CoppeliasimControl import CoppeliasimControl
from ModernRoboticsIK import ModernRoboticsIK
import modern_robotics as mr

if __name__ == '__main__':

    joint_list = ['/piper/joint1', '/piper/joint2', '/piper/joint3', '/piper/joint4', '/piper/joint5', '/piper/joint6']
    tool_list = ['/piper/joint7', '/piper/joint8']
    sim = CoppeliasimControl(joint_list, tool_list)

    shared_memory_name = "PiPER"
    arm_topic = "right/"

    client = MQTT_Client(arm_topic, "local")

    robot_model = 'piper_agilex'
    piperik = ModernRoboticsIK(robot_model)
    ik_mode = 'inSpace'

    try:
        client.start_mqtt()
        time.sleep(0.1)
        prev_vr_time = 0
        while True:
            vr_time, p_diff, R_relative = client.get_vr_diff()

            if vr_time != prev_vr_time:
                print(vr_time, p_diff, R_relative)
                current_joint_position = sim.get_joint_position()
                # print("joint_position", joint_position)

                T = piperik.fk(current_joint_position, ik_mode)
                R, p = mr.TransToRp(T)

                newP = p + p_diff
                R_screw, R_theta = piperik.relativeRMatrixtoScrewAxis(R_relative)
                R_screw_world = [-R_screw[2], -R_screw[0], R_screw[1]]
                R_relative_world = piperik.ScrewAxisToRelativeRMatrix(R_screw_world, R_theta)
                newR_inSpace = np.dot(R_relative_world, R)

                newT = mr.RpToTrans(newR_inSpace, newP)

                target_joint, ik_state = piperik.IK_joint_velocity_limit(newT, current_joint_position, ik_mode)  # 6Dof robot

                sim.send_joint_position(target_joint)
                prev_vr_time = vr_time

                timestamp = int(time.time()*1000)
                robot_state_msg = {
                    "time": timestamp,
                    "state": "initialize",
                    "model": "agilex_piper",
                    "joint_feedback": sim.get_joint_position(),
                }
                client.publish_message(robot_state_msg)
                time.sleep(0.025)

    except KeyboardInterrupt:
        print("MQTT Recv Stopped")
        sys.exit(0)
    except Exception as e:
        print("MQTT Recv Error:", e)
        sys.exit(1)
