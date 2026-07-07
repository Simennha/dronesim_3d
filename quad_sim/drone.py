import math
import parameters as p


class Drone:
    def __init__(self, x=0.0, y=0.0, z=0.0, phi=0.0, theta=0.0, psi=0.0):
        self.x = x
        self.y = y
        self.z = z

        self.phi = phi      # roll  (rotation about body x-axis)
        self.theta = theta  # pitch (rotation about body y-axis)
        self.psi = psi      # yaw   (rotation about world z-axis)

        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0

        self.phi_dot = 0.0
        self.theta_dot = 0.0
        self.psi_dot = 0.0

        self.w_front = 0.0
        self.w_right = 0.0
        self.w_back = 0.0
        self.w_left = 0.0

    def step(self, w_front, w_right, w_back, w_left, dt=p.DT):
        self.w_front = w_front
        self.w_right = w_right
        self.w_back = w_back
        self.w_left = w_left

        F_front = p.K * w_front ** 2
        F_right = p.K * w_right ** 2
        F_back = p.K * w_back ** 2
        F_left = p.K * w_left ** 2

        F_total = F_front + F_right + F_back + F_left

        torque_roll = p.L * (F_left - F_right)
        torque_pitch = p.L * (F_back - F_front)
        torque_yaw = (p.B / p.K) * (F_front + F_back - F_left - F_right)

        phi, theta, psi = self.phi, self.theta, self.psi
        sin_phi, cos_phi = math.sin(phi), math.cos(phi)
        sin_theta, cos_theta = math.sin(theta), math.cos(theta)
        sin_psi, cos_psi = math.sin(psi), math.cos(psi)

        # direction of body z-axis (thrust axis) expressed in world frame
        dir_x = sin_theta * cos_phi * cos_psi + sin_phi * sin_psi
        dir_y = sin_theta * cos_phi * sin_psi - sin_phi * cos_psi
        dir_z = cos_phi * cos_theta

        Fx_world = F_total * dir_x
        Fy_world = F_total * dir_y
        Fz_world = F_total * dir_z - p.MASS * p.GRAVITY

        ax = Fx_world / p.MASS
        ay = Fy_world / p.MASS
        az = Fz_world / p.MASS

        alpha_phi = torque_roll / p.IXX
        alpha_theta = torque_pitch / p.IYY
        alpha_psi = torque_yaw / p.IZZ

        self.vx += ax * dt
        self.vy += ay * dt
        self.vz += az * dt

        self.phi_dot += alpha_phi * dt
        self.theta_dot += alpha_theta * dt
        self.psi_dot += alpha_psi * dt

        self.x += self.vx * dt
        self.y += self.vy * dt
        self.z += self.vz * dt

        self.phi += self.phi_dot * dt
        self.theta += self.theta_dot * dt
        self.psi += self.psi_dot * dt

        self.psi = (self.psi + math.pi) % (2 * math.pi) - math.pi

        if self.z < 0.0:
            self.z = 0.0
            if self.vz < 0.0:
                self.vz = 0.0

    def get_state(self):
        return {
            "x": self.x, "y": self.y, "z": self.z,
            "phi": self.phi, "theta": self.theta, "psi": self.psi,
            "vx": self.vx, "vy": self.vy, "vz": self.vz,
            "phi_dot": self.phi_dot, "theta_dot": self.theta_dot, "psi_dot": self.psi_dot,
            "w_front": self.w_front, "w_right": self.w_right,
            "w_back": self.w_back, "w_left": self.w_left,
        }
