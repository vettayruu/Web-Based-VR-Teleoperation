import time
import modern_robotics as mr
import numpy as np
from PIPERControl import PIPERControl
from ModernRoboticsIK import ModernRoboticsIK
from YOLOSegPose import YOLOSegPose
import cv2

# np.set_printoptions(precision=5, suppress=True)

def undistortion(img):
    K = np.array([[788.41415049, 0., 655.01692926],
                  [0., 787.3765135, 357.82862631],
                  [0., 0., 1.]], dtype=np.float32)
    # 畸变系数 dist:
    dist = np.array([[-0.3506601, 0.18558038, -0.00065609, 0.00100313, -0.05786136]], dtype=np.float32)

    h, w = img.shape[:2]
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(K, dist, (w, h), 0, (w, h))
    undistorted = cv2.undistort(img, K, dist, None, newcameramtx)
    return undistorted

robot_model = 'piper_agilex'
piperik = ModernRoboticsIK(robot_model)
mode = "inSpace"
# mode = "inBody"

can_port = "can0"
piper_interface = PIPERControl(can_port)
piper_interface.connect()
time.sleep(0.1)

Tf = 0.025
dt = 0.005
N = 5
method = 5
Kp = 0.18
Kd = 0.0002
prev_error = np.zeros(6)

Kp_v = 0.20
Kd_v = 0.0001

prev_mean_offset_cy = prev_mean_offset_cx = 0
prev_offset_theta = 0

if __name__ == "__main__":
    # send initial pose
    model_path_seg = "best.pt"
    # yolo_model = YOLO(model_path_seg)

    yolo_seg_left = YOLOSegPose(model_path_seg)
    yolo_seg_right = YOLOSegPose(model_path_seg)

    cap = cv2.VideoCapture(0)
    if cap.isOpened() == 0:
        exit(-1)

    # Set the video resolution to HD720 (2560*720)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 2560)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    while True:
        # Get a new frame from camera
        retval, frame = cap.read()
        # Extract left and right images from side-by-side
        left_right_image = np.split(frame, 2, axis=1)

        left_image = left_right_image[0]
        right_image = left_right_image[1]

        left_image = undistortion(left_image)
        right_image = undistortion(right_image)

        left_record = left_image.copy()
        right_record = right_image.copy()

        yolo_seg_left.visualize(left_image)
        yolo_seg_right.visualize(right_image)

        cv2.imshow("left", left_image)
        cv2.imshow("right", right_image)

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
        offset_theta = (offset_left + offset_right)/2
        offset_mse = (offset_left**2 + offset_right**2)/2
        rmse_offset = np.sqrt(offset_mse)
        # print("theta offset", rmse_offset)

        offset_cx_l = (right_arm_cx_l - pack_cx_l)/1000
        offset_cx_r = (right_arm_cx_r - pack_cx_r)/1000
        mean_offset_cx = (offset_cx_l + offset_cx_r)/2
        mse_offset_cx = (offset_cx_l**2 + offset_cx_r**2) / 2
        rmse_offset_cx = np.sqrt(mse_offset_cx)
        # print("cx offset", rmse_offset_cx)

        offset_cy_r = (right_arm_cy_r - pack_cy_r)/1000
        offset_cy_l = (right_arm_cy_l - pack_cy_l)/1000
        mean_offset_cy = (offset_cy_l + offset_cy_r)/2
        mse_offset_cy = (offset_cy_l**2 + offset_cy_r**2) / 2
        rmse_offset_cy = np.sqrt(mse_offset_cy)
        # print("cy offset", rmse_offset_cy)

        total_loss = (rmse_offset + rmse_offset_cx + rmse_offset_cy)/3
        print("total loss", total_loss)

        d_offset_theta = (offset_theta - prev_offset_theta)/Tf
        d_mean_offset_cy = (mean_offset_cy - prev_mean_offset_cy)/Tf
        d_mean_offset_cx = (mean_offset_cx - prev_mean_offset_cx)/Tf

        prev_offset_theta = offset_theta
        prev_mean_offset_cy = mean_offset_cy
        prev_mean_offset_cx = mean_offset_cx

        # Get current pose
        theta_start = piper_interface.get_joint_feedback_mr()
        T = piperik.fk(theta_start, mode)
        R, p = mr.TransToRp(T)

        # Position
        p[0] += Kp_v * mean_offset_cy + Kd_v * d_mean_offset_cy
        p[1] += Kp_v * mean_offset_cx + Kd_v * d_mean_offset_cx
        # Rotation
        yaw = Kp_v * offset_theta + Kd_v * d_offset_theta
        R = piperik.z_axis_rotate(R, yaw, mode)
        # Update target joint position
        T_sd = mr.RpToTrans(R, p)
        theta_end, status = piperik.IK_joint_velocity_limit(T_sd, theta_start, mode)

        # PD control
        error = theta_end - theta_start
        d_error = (error - prev_error) / Tf
        mse = np.mean(error ** 2)  # Mean Square Error
        rmse = np.sqrt(mse)

        theta_target = theta_start + Kp * error + Kd * d_error
        prev_error = error.copy()

        key_code = cv2.waitKey(int(Tf*1000)) & 0xFF
        if key_code != 255:
            try:
                key = chr(key_code)
            except ValueError:
                key = ''

            if key == 'm' and total_loss > 0.001:
                # Trajectory Plan
                if rmse > 0.001:
                    theta_traj = mr.JointTrajectory(theta_start, theta_target, Tf, N, method)
                    for theta in theta_traj:
                        piper_interface.joint_control_offset(theta, 10)
                        time.sleep(dt)

            elif key == 'r':
                piper_interface.right_arm_work_position()

            elif key == 's':
                timestamp = int(time.time()*1000)
                cv2.imwrite(f'./record/left/zed_left_{timestamp}.jpg', left_record)
                cv2.imwrite(f'./record/right/zed_right_{timestamp}.jpg', right_record)

            elif key == '\x1b':  # ESC (27)
                print("Exit program.")
                break





