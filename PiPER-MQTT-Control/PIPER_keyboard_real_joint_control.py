import time
import modern_robotics as mr
import numpy as np
from PIPERControl import PIPERControl
from ModernRoboticsIK import ModernRoboticsIK
import cv2

np.set_printoptions(precision=4, suppress=True)

step = 0.005

can_port = "can0"
piper_interface = PIPERControl(can_port)
piper_interface.connect()
time.sleep(0.1)

np.set_printoptions(precision=5, suppress=True)

robot_model = 'piper_agilex'
piperik = ModernRoboticsIK(robot_model)

w, h = 640, 480
cv2.namedWindow("Keyboard Listener", cv2.WINDOW_NORMAL)
cv2.resizeWindow("Keyboard Listener", w, h)
img = 255 * np.ones((h, w, 3), dtype=np.uint8)

if __name__ == "__main__":
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

                joint_position = piper_interface.get_joint_feedback()
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
                    # sim.send_joint_position(theta_initial)
                    print("reset")
                    continue

                elif key == '\x1b':  # ESC (27)
                    print("Exit program.")
                    break

                piper_interface.joint_control(joint_position, 5)

    except KeyboardInterrupt:
        cv2.destroyAllWindows()
        print("\n退出控制")

