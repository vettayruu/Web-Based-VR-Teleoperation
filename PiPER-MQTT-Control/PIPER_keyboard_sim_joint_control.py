import time
import modern_robotics as mr
import numpy as np
from CoppeliasimControl import CoppeliasimControl
from ModernRoboticsIK import ModernRoboticsIK
import cv2

np.set_printoptions(precision=5, suppress=True)

robot_model = 'piper_agilex'
piperik = ModernRoboticsIK(robot_model)

joint_list = ['/piper/joint1', '/piper/joint2', '/piper/joint3', '/piper/joint4', '/piper/joint5', '/piper/joint6']
tool_list = ['/piper/joint7', '/piper/joint8']
sim = CoppeliasimControl(joint_list, tool_list)

w, h = 640, 480
cv2.namedWindow("Keyboard Listener", cv2.WINDOW_NORMAL)
cv2.resizeWindow("Keyboard Listener", w, h)
img = 255 * np.ones((h, w, 3), dtype=np.uint8)

if __name__ == "__main__":
    # send initial pose
    theta_initial = np.array([0.05403105546934629, 0.5266943413228493, 2.3184564294687373, 1.515287347742791, 1.1937675559681376, 0.30173671821985093])
    # [0.1217, 0.5318, -0.9902, 1.1096, 1.2116, 0.53602]
    # [0.1217, 0.5318, 1.44144, 0, 1.22586, 0]
    sim.send_joint_position(theta_initial)

    step = 0.005

    # keyboard control
    try:
        while True:
            cv2.putText(img, "Joint 1: Q/A, Joint 2: W/S, Joint 3: E/D ", (30, 100),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
            cv2.putText(img, "Joint 4: U/JA, Joint 5: I/K, Joint 6: O/L ", (30, 130),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
            cv2.putText(img, "ESC to exit", (30, 160),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
            cv2.imshow("Keyboard Listener", img)

            key_code = cv2.waitKey(1) & 0xFF
            if key_code != 255:
                try:
                    key = chr(key_code)
                except ValueError:
                    key = ''

                joint_position = sim.get_joint_position()
                print("joint_position:", joint_position)

                if key == 'q':
                    joint_position[0] += step
                elif key == 'a':
                    joint_position[0] -= step
                elif key == 'w':
                    joint_position[1] += step
                elif key == 's':
                    joint_position[1] -= step
                elif key == 'e':
                    joint_position[2] += step
                elif key == 'd':
                    joint_position[2] -= step

                elif key == 'u':
                    joint_position[3] += step
                elif key == 'j':
                    joint_position[3] -= step
                elif key == 'i':
                    joint_position[4] += step
                elif key == 'k':
                    joint_position[4] -= step
                elif key == 'o':
                    joint_position[5] += step
                elif key == 'l':
                    joint_position[5] -= step

                elif key == 'r':
                    sim.send_joint_position(theta_initial)
                    print("reset")
                    continue

                elif key == '\x1b':  # ESC (27)
                    print("Exit program.")
                    break

                sim.send_joint_position(joint_position)

    except KeyboardInterrupt:
        cv2.destroyAllWindows()
        print("\n退出控制")


