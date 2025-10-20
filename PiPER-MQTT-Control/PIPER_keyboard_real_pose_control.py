import time
import modern_robotics as mr
import numpy as np
from PIPERControl import PIPERControl
from ModernRoboticsIK import ModernRoboticsIK
import cv2

np.set_printoptions(precision=5, suppress=True)

robot_model = 'piper_agilex'
piperik = ModernRoboticsIK(robot_model)
# mode = "inSpace"
mode = "inBody"

can_port = "can0"
piper_interface = PIPERControl(can_port)
piper_interface.connect()
time.sleep(0.1)

w, h = 640, 480
cv2.namedWindow("Keyboard Listener", cv2.WINDOW_NORMAL)
cv2.resizeWindow("Keyboard Listener", w, h)
img = 255 * np.ones((h, w, 3), dtype=np.uint8)

if __name__ == "__main__":
    # send initial pose
    p_step = 0.001
    R_step = 0.005

    # keyboard control
    try:
        while True:
            cv2.putText(img, "Position: W/A/S/D/Q/E", (30, 70),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
            cv2.putText(img, "Rotation: I/K (X-axis), J/L (Y-axis), U/O (Z-axis)", (30, 100),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
            cv2.putText(img, "Reset: R", (30, 130),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
            cv2.putText(img, "Mode Change: M", (30, 160),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
            cv2.putText(img, "ESC to exit", (30, 190),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
            cv2.imshow("Keyboard Listener", img)

            joint_position = piper_interface.get_joint_feedback_mr()
            T = piperik.fk(joint_position, mode)
            R, p = mr.TransToRp(T)
            yaw = pitch = roll = 0

            key_code = cv2.waitKey(1) & 0xFF
            if key_code != 255:
                try:
                    key = chr(key_code)
                except ValueError:
                    key = ''

                if key == 'w':
                    p[0] += p_step
                elif key == 's':
                    p[0] -= p_step
                elif key == 'a':
                    p[1] += p_step
                elif key == 'd':
                    p[1] -= p_step
                elif key == 'e':
                    p[2] += p_step
                elif key == 'q':
                    p[2] -= p_step

                elif key == 'u':
                    yaw += R_step
                    R = piperik.z_axis_rotate(R, yaw, mode)
                elif key == 'j':
                    yaw -= R_step
                    R = piperik.z_axis_rotate(R, yaw, mode)
                elif key == 'i':
                    pitch += R_step
                    R = piperik.y_axis_rotate(R, pitch, mode)
                elif key == 'k':
                    pitch -= R_step
                    R = piperik.y_axis_rotate(R, pitch, mode)
                elif key == 'o':
                    roll += R_step
                    R = piperik.x_axis_rotate(R, roll, mode)
                elif key == 'l':
                    roll -= R_step
                    R = piperik.x_axis_rotate(R, roll, mode)

                elif key == 'r':
                    # sim.send_joint_position(theta_initial)
                    print("reset")
                    continue

                elif key == 'm':
                    if mode == "inSpace":
                        mode = "inBody"
                    else:
                        mode = "inSpace"
                    print(f"Mode switched to: {mode}")
                    continue

                elif key == '\x1b':  # ESC (27)
                    print("Exit program.")
                    break

                T_sd = mr.RpToTrans(R, p)

                thetaBody, status = piperik.IK_joint_velocity_limit(T_sd, joint_position, mode)
                piper_interface.joint_control_offset(thetaBody, 5)
                print(thetaBody)

                # time.sleep(0.05)

    except KeyboardInterrupt:
        cv2.destroyAllWindows()
        print("\n退出控制")


