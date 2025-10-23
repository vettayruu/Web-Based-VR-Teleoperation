import time
import modern_robotics as mr
import numpy as np
from PIPERControl import PIPERControl
from ModernRoboticsIK import ModernRoboticsIK
from YOLOSegPose import YOLOSegPose
from MQTT_Client import MQTT_Client
from ZEDStereoSender import ZEDStereoSender
import cv2

np.set_printoptions(precision=6, suppress=True)

class Undistortion():
    def __init__(self):
        self.K = np.array([[788.41415049, 0., 655.01692926],
                                 [0., 787.3765135, 357.82862631],
                                 [0., 0., 1.]], dtype=np.float32)
        self.dist = np.array([[-0.3506601, 0.18558038, -0.00065609, 0.00100313, -0.05786136]], dtype=np.float32)
        self.newcameramtx = None

    def get_cameramtx(self, img):
        h, w = img.shape[:2]
        self.newcameramtx, roi = cv2.getOptimalNewCameraMatrix(self.K, self.dist, (w, h), 0, (w, h))
        return

    def show(self, img):
        undistorted = cv2.undistort(img, self.K, self.dist, None, self.newcameramtx)
        return undistorted

robot_model = 'piper_agilex'
piperik = ModernRoboticsIK(robot_model)
mode = "inSpace"
# mode = "inBody"

can_port = "can0"
piper= PIPERControl(can_port)
piper.connect()
time.sleep(0.5)

name_sh = "RightArm"
joint_topic = "joint"
tool_topic = "tool"
mqtt_mode = "local"
client = MQTT_Client(joint_topic, tool_topic, mqtt_mode)

undist = Undistortion()

signaling_urls = ["wss://sora2.uclab.jp/signaling"]
left_channel = "sora_liust_left"
right_channel = "sora_liust_right"

Tf = 0.025
Kp_v = 0.095
Kd_v = 0.0008

loss_threshold = 0.010
prev_loss_mag = 0

prev_time = time.time()
frame_count = 0

if __name__ == "__main__":
    # send initial pose
    model_path_seg = "best.pt"

    yolo_seg_left = YOLOSegPose(model_path_seg)
    yolo_seg_right = YOLOSegPose(model_path_seg)

    cap = cv2.VideoCapture(0)
    if cap.isOpened() == 0:
        exit(-1)

    # Set the video resolution to HD720 (2560*720)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 2560)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    stereo_sender = ZEDStereoSender(signaling_urls, left_channel, right_channel)
    stereo_sender.initialize_sora_connections()

    # connect once
    stereo_sender.left_sendonly.connect()
    stereo_sender.right_sendonly.connect()

    client.start_mqtt()
    flag_shm = client.verify_shared_memory(name_sh)
    fps = 0

    while flag_shm:
        # Get a new frame from camera
        retval, frame = cap.read()
        left_right_image = np.split(frame, 2, axis=1)

        frame_count += 1
        if time.time() - prev_time > 1.0:
            fps = frame_count
            frame_count = 0
            prev_time = time.time()

        left_image = left_right_image[0]
        right_image = left_right_image[1]

        left_image = undist.show(left_image)
        right_image = undist.show(right_image)

        left_record = left_image.copy()
        right_record = right_image.copy()

        shared_control_flag = client.get_shared_control_flag()
        if shared_control_flag == 1:
            yolo_seg_left.visualize(left_image)
            yolo_seg_right.visualize(right_image)

            # object
            pack_cx_l, pack_cy_l, pack_theta_l = yolo_seg_left.get_pack_pose()
            pack_cx_r, pack_cy_r, pack_theta_r = yolo_seg_right.get_pack_pose()

            # # left_arm
            # left_arm_cx_l, left_arm_cy_l, left_arm_theta_l = yolo_seg_left.get_tip_pose_left()
            # left_arm_cx_r, left_arm_cy_r, left_arm_theta_r = yolo_seg_right.get_tip_pose_left()
            # print(f"left_arm_pose_l: {left_arm_cx_l}, {left_arm_cy_l}, {np.rad2deg(left_arm_theta_l)}")
            # print(f"left_arm_pose_r: {left_arm_cx_r}, {left_arm_cy_r}, {np.rad2deg(left_arm_theta_r)}")

            # right_arm
            right_arm_cx_l, right_arm_cy_l, right_arm_theta_l = yolo_seg_left.get_tip_pose_right()
            right_arm_cx_r, right_arm_cy_r, right_arm_theta_r = yolo_seg_right.get_tip_pose_right()

            offset_left = right_arm_theta_l - pack_theta_l
            offset_right = right_arm_theta_r - pack_theta_r
            mean_offset_theta = (offset_left + offset_right)/2

            offset_cx_l = (right_arm_cx_l - pack_cx_l)/1000
            offset_cx_r = (right_arm_cx_r - pack_cx_r)/1000
            mean_offset_cx = (offset_cx_l + offset_cx_r)/2

            offset_cy_r = (right_arm_cy_r - pack_cy_r)/1000
            offset_cy_l = (right_arm_cy_l - pack_cy_l)/1000
            mean_offset_cy = (offset_cy_l + offset_cy_r)/2

            loss = [mean_offset_cy, mean_offset_cx, mean_offset_theta]
            loss_hat, loss_mag = mr.AxisAng3(loss)

            d_loss_mag = (loss_mag - prev_loss_mag)/Tf
            prev_loss_mag = loss_mag

            loss_theta = Kp_v * loss_mag + Kd_v * d_loss_mag

            # VR Control
            shared_control_signal = client.get_shared_control_signal()

            if shared_control_signal == 1 and loss_mag > loss_threshold:
                # Get current pose
                theta_start = piper.get_joint_feedback_mr()
                T = piperik.fk(theta_start, mode)
                R, p = mr.TransToRp(T)

                # Position
                p[0] += loss_hat[0] * loss_theta
                p[1] += loss_hat[1] * loss_theta
                # Rotation
                yaw = loss_hat[2] * loss_theta
                R = piperik.z_axis_rotate(R, yaw, mode)

                # Update target joint position
                T_sd = mr.RpToTrans(R, p)
                theta_end, status = piperik.IK_joint_velocity_limit(T_sd, theta_start, mode)

                data_sh = client.read_shared_memory(name_sh)
                data_sh[8:14] = theta_end
                client.write_shared_memory(name_sh, data_sh)

                joint_feedback = piper.get_joint_feedback_mr()
                timestamp = int(time.time() * 1000)
                robot_state_msg = {
                    "time": timestamp,
                    "state": "initialize",
                    "model": "agilex_piper",
                    "joint_feedback": joint_feedback,
                }
                client.publish_message(robot_state_msg)

            elif shared_control_signal == -1:
                initial_joint_position = np.array([-0.09306611111164882, 0.47493111766136753, 1.9607769506067685, 0.9579791111166461,
                 1.2210587777848327, 0.5382308888919987])

                data_sh = client.read_shared_memory(name_sh)
                data_sh[8:14] = initial_joint_position
                client.write_shared_memory(name_sh, data_sh)

                joint_feedback = piper.get_joint_feedback_mr()
                timestamp = int(time.time() * 1000)
                robot_state_msg = {
                    "time": timestamp,
                    "state": "initialize",
                    "model": "agilex_piper",
                    "joint_feedback": joint_feedback,
                }
                client.publish_message(robot_state_msg)


        # cv2.putText(left_image, f"Shared Control State: {shared_control_signal}", (20, 30),
        #             cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.putText(right_image, f"fps: {fps}", (20, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.putText(right_image, f"Shared Control On", (20, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.putText(right_image, f"Shared Control State: {shared_control_signal}", (20, 90),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.putText(right_image, f"Loss Visual: {loss_mag: .4f} ", (20, 120),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            stereo_sender.send_frames(left_image, right_image)

            # cv2.imshow("left", left_image)
            # cv2.imshow("right", right_image)

        # elapsed = time.time() - start
        # print("elapsed", elapsed)
        else:
            cv2.putText(right_image, f"fps: {fps}", (20, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.putText(right_image, f"Shared Control OFF", (20, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            stereo_sender.send_frames(left_image, right_image)

        # Keyboard Control
        key_code = cv2.waitKey(1) & 0xFF
        if key_code != 255:
            try:
                key = chr(key_code)
            except ValueError:
                key = ''

            if key == 's':
                timestamp = int(time.time()*1000)
                cv2.imwrite(f'./record/left/zed_left_{timestamp}.jpg', left_record)
                cv2.imwrite(f'./record/right/zed_right_{timestamp}.jpg', right_record)

            elif key == '\x1b':  # ESC (27)
                # client.close_shared_memory(name_sh)
                print("Exit program.")
                break

    cap.release()
    cv2.destroyAllWindows()





