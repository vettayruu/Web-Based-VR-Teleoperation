from RobotKinematics import RobotKinematics
import modern_robotics as mr
import numpy as np

class STATE_CODES:
    NORMAL = 0
    VELOCITY_LIMIT = 1
    JOINT_LIMIT = 2
    IK_FAILED = 3

class ModernRoboticsIK():
    def __init__(self, robot_model):
        rk = RobotKinematics(robot_model)

        M = rk.get_M()
        Slist = rk.get_Slist()
        Blist = np.dot(mr.Adjoint(mr.TransInv(M)), Slist)
        jointLimits = rk.get_jointLimits()

        self.robotParams = {
            "M": M,
            "Slist": Slist,
            "Blist": Blist,
            "jointLimits": jointLimits
        }

        self.dt = 0.01667

    def fk(self, thetalist, mode):
        M = self.robotParams["M"]
        Slist = self.robotParams["Slist"]
        Blist = self.robotParams["Blist"]
        if mode == "inSpace":
            T = mr.FKinSpace(M, Slist, thetalist)
        elif mode == "inBody":
            T = mr.FKinBody(M, Blist, thetalist)
        return T

    def IK_joint_velocity_limit(self, T_sd, theta_body, VR_Control_Mode):
        """
        限制关节速度和角度的逆运动学求解器。
        输入:
            T_sd: 目标末端位姿 (4x4 numpy array)
            theta_body: 当前关节角 (numpy array)
            VR_Control_Mode: 'inBody' 或 'inSpace'
        输出:
            new_theta_body: 更新后的关节角
            error_code: 状态码
        """

        max_joint_velocity = 5.5   # 最大关节速度限制

        M = self.robotParams["M"]
        Slist = self.robotParams["Slist"]
        Blist = self.robotParams["Blist"]
        jointLimits = self.robotParams["jointLimits"]

        error_code = STATE_CODES.NORMAL

        # ---------- 求逆运动学 ----------
        if VR_Control_Mode == 'inBody':
            thetalist_sol, success = mr.IKinBody(Blist, M, T_sd, theta_body, 1e-5, 1e-5)
        elif VR_Control_Mode == 'inSpace':
            thetalist_sol, success = mr.IKinSpace(Slist, M, T_sd, theta_body, 1e-5, 1e-5)
        else:
            raise ValueError("Invalid VR_Control_Mode: must be 'inBody' or 'inSpace'")

        if success:
            # 限制角度范围
            thetalist_sol_limited = np.array([
                np.clip(theta, jointLimits[i]['min'], jointLimits[i]['max'])
                for i, theta in enumerate(thetalist_sol)
            ])

            # 检查是否触及关节限位
            joint_limited = any(
                thetalist_sol_limited[i] == jointLimits[i]['min'] or
                thetalist_sol_limited[i] == jointLimits[i]['max']
                for i in range(len(thetalist_sol_limited))
            )

            # 计算关节速度
            delta_theta = thetalist_sol_limited - theta_body
            d_theta = delta_theta / self.dt

            ometahat, theta_mag = mr.AxisAng3(d_theta)

            joint_velocity = np.clip(theta_mag, 0, max_joint_velocity)
            new_theta_body = theta_body + ometahat * joint_velocity * self.dt

            # 限速判断
            if joint_velocity == max_joint_velocity:
                print("⚠️ Joint Velocity Limit Reached")
                error_code = STATE_CODES.VELOCITY_LIMIT
                return new_theta_body, error_code

            # 角度限位判断
            elif joint_limited:
                print("⚠️ Joint Angle Limit Reached")
                error_code = STATE_CODES.JOINT_LIMIT
                # return_theta_body = theta_body + ometahat * joint_velocity_return * dt
                return new_theta_body, error_code

            else:
                return new_theta_body, error_code

        else:
            print("⚠️ IK failed to converge")
            error_code = STATE_CODES.IK_FAILED
            return theta_body, error_code

    def z_axis_rotate(self, R, theta, mode):
        omega = np.array([0., 0., 1.])
        so3mat = mr.VecToso3(omega * theta)
        Rz = mr.MatrixExp3(so3mat)

        if mode == "inSpace":
            R_new = Rz @ R
        elif mode == "inBody":
            R_new = R @ Rz

        return R_new

    def y_axis_rotate(self, R, theta, mode):
        omega = np.array([0., 1., 0.])
        so3mat = mr.VecToso3(omega * theta)
        Ry = mr.MatrixExp3(so3mat)

        if mode == "inSpace":
            R_new = Ry @ R
        elif mode == "inBody":
            R_new = R @ Ry

        return R_new

    def x_axis_rotate(self, R, theta, mode):
        omega = np.array([1., 0., 0.])
        so3mat = mr.VecToso3(omega * theta)
        Ry = mr.MatrixExp3(so3mat)

        if mode == "inSpace":
            R_new = Ry @ R
        elif mode == "inBody":
            R_new = R @ Ry

        return R_new

    def relativeRMatrixtoScrewAxis(self, relativeR):
        # Calculate so(3) matrix using MatrixLog3
        so3mat = mr.MatrixLog3(relativeR)

        # Extract omega_theta vector from so(3) matrix
        omega_theta = mr.so3ToVec(so3mat)

        # Calculate magnitude (theta) using JavaScript array operations
        theta = np.sqrt(
            omega_theta[0] * omega_theta[0] +
            omega_theta[1] * omega_theta[1] +
            omega_theta[2] * omega_theta[2]
        )

        if (theta < 1e-6):
            # If theta is very small, return zero vector
            omega_hat = [0, 0, 0]
        else:
            # Normalize omega_theta to get unit axis vector
            omega_hat = [
                omega_theta[0] / theta,
                omega_theta[1] / theta,
                omega_theta[2] / theta
            ]

        return omega_hat, theta

    def ScrewAxisToRelativeRMatrix(self, omega_hat, theta):
        omega_theta =[
            omega_hat[0] * theta,
            omega_hat[1] * theta,
            omega_hat[2] * theta
            ]

        so3mat = mr.VecToso3(omega_theta)
        R_relative = mr.MatrixExp3(so3mat)

        return R_relative

