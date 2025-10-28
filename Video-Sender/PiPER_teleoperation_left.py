import sys
import subprocess
import os
import atexit

import time
import numpy as np
import multiprocessing.shared_memory as sm

from PIPERControl import PIPERControl
from MQTT_Client import MQTT_Client
from MQTT_Client_2 import MQTT_Client
import modern_robotics as mr

# To run the code, activate can bus at first: bash can_activate.sh can0 1000000

if __name__ == "__main__":
    # Open Monitor
    # monitor_script = os.path.join(os.getcwd(), "MQTT_Robot_Monitor_UI.py")
    # monitor_proc = subprocess.Popen([sys.executable, monitor_script])
    # print("Monitor UI launched.")
    #
    # def cleanup():
    #     if monitor_proc.poll() is None:
    #         print("Terminating Monitor UI...")
    #         monitor_proc.terminate()
    #         try:
    #             monitor_proc.wait(timeout=2)
    #         except subprocess.TimeoutExpired:
    #             monitor_proc.kill()
    #
    # atexit.register(cleanup)

    can_port = "can1"
    piper = PIPERControl(can_port)
    piper.connect()
    piper.enable()
    time.sleep(1)

    name = "LeftArm"
    joint_topic = "joint"
    tool_topic = "tool"
    arm_topic = 'left/'
    mode = "local"
    # client = MQTT_Client(joint_topic, tool_topic, mode)
    client = MQTT_Client(arm_topic, mode)

    # Control Parameter
    Tf = 0.025
    dt = 0.005
    N = 5
    method = 5
    Kp = 0.75
    Kd = 0.0015

    prev_error = np.zeros(6)

    try:
        client.create_shared_memory(name)
        client.start_mqtt()
        time.sleep(0.1)

        """
        Check Robot State
        """
        # Update joint message
        arr = client.get_shared_memory()
        thetaBody = arr[8:14].astype(float)  # 6Dof robot
        thetaTool = arr[15].astype(float)

        # msg = piper.GetArmJointMsgs()
        joint_feedback = piper.get_joint_feedback_mr()
        arr[0:6] = joint_feedback

        timestamp = int(time.time()*1000)

        # Send robot current state to MQTT
        robot_state_msg = {
            "time": timestamp,
            "state_left": "initialize",
            "model_left": "agilex_piper",
            "joint_feedback_left": joint_feedback,
        }
        client.publish_message(robot_state_msg)
        time.sleep(1.0)

        print("shared memory:", arr)

        a = arr[0:6]
        b = arr[8:14]
        equal = np.allclose(a, b)

        if equal:
            robot_state_msg = {
                "state_left": "ready",
            }
            client.publish_message(robot_state_msg)
            print("Robot Ready.")
        else:
            print("Robot Not Ready. Please check VR control communication.")

        while equal:
            # Update joint message
            thetaBody = arr[8:14].astype(float)  # 6Dof robot
            thetaBody = [round(x, 4) for x in thetaBody]
            thetaBody = np.array(thetaBody)

            thetaTool = arr[15].astype(float)

            # Get joint feedback
            joint_feedback = piper.get_joint_feedback_mr()
            joint_feedback = [round(x, 4) for x in joint_feedback]
            joint_feedback = np.array(joint_feedback)
            arr[0:6] = joint_feedback

            error = thetaBody - joint_feedback
            d_error = (error - prev_error) / Tf
            mse = np.mean(error ** 2)  # Mean Square Error
            rmse = np.sqrt(mse)

            # PD control
            control_signal = joint_feedback + Kp * error + Kd * d_error
            prev_error = error.copy()

            # Trajectory Plan
            theta_current = joint_feedback
            theta_target =control_signal
            if rmse > 0.0015:
                theta_traj = mr.JointTrajectory(theta_current, theta_target, Tf, N, method)

                for theta in theta_traj:
                    piper.joint_control_offset(theta, 60)

                    finger_pos = ((thetaTool) * 0.85) + 0.4  # /mm
                    piper.gripper_control(finger_pos, 1000)

                    time.sleep(dt)
            else:
                finger_pos = ((thetaTool) * 0.85) + 0.4  # /mm
                piper.gripper_control(finger_pos, 1000)
                time.sleep(dt)

    except KeyboardInterrupt:
        print("MQTT Recv Stopped")
        robot_state_msg = {
            "state": "stop",
        }
        client.publish_message(robot_state_msg)
        sys.exit(0)
    except Exception as e:
        print("MQTT Recv Error:", e)
        robot_state_msg = {
            "state": "error",
        }
        client.publish_message(robot_state_msg)
        sys.exit(1)