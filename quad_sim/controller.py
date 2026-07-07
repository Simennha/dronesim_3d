import math
import parameters as p


class PID:
    def __init__(self, kp, ki, kd, output_min=None, output_max=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.output_min = output_min
        self.output_max = output_max

        self.integral = 0.0
        self.prev_error = 0.0

    def reset(self, error=0.0):
        self.integral = 0.0
        self.prev_error = error

    def update(self, error, dt):
        proportional_term = self.kp * error

        self.integral += error * dt
        integral_term = self.ki * self.integral

        derivative = (error - self.prev_error) / dt
        derivative_term = self.kd * derivative

        output = proportional_term + integral_term + derivative_term
        if self.output_max is not None and output > self.output_max:
            output = self.output_max
            if self.ki != 0:
                self.integral = (self.output_max - proportional_term - derivative_term) / self.ki
        elif self.output_min is not None and output < self.output_min:
            output = self.output_min
            if self.ki != 0:
                self.integral = (self.output_min - proportional_term - derivative_term) / self.ki

        self.prev_error = error
        return output


def wrap_angle(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


class PositionController:
    """Cascaded PID: altitude -> thrust, xy position -> tilt angle, tilt angle -> torque."""

    def __init__(self, x_target=0.0, y_target=0.0, z_target=0.0, psi_target=0.0):
        self.x_target = x_target
        self.y_target = y_target
        self.z_target = z_target
        self.psi_target = psi_target

        self.pid_z = PID(kp=14.0, ki=0, kd=13.0,
                          output_min=-3 * p.MASS * p.GRAVITY,
                          output_max=6 * p.MASS * p.GRAVITY)

        self.pid_x = PID(kp=1.0, ki=0, kd=0.9,
                          output_min=-p.MAX_TILT_ANGLE, output_max=p.MAX_TILT_ANGLE)
        self.pid_y = PID(kp=1.0, ki=0, kd=0.9,
                          output_min=-p.MAX_TILT_ANGLE, output_max=p.MAX_TILT_ANGLE)

        self.pid_pitch = PID(kp=8.0, ki=0, kd=1.4,
                              output_min=-p.TORQUE_PITCH_MAX, output_max=p.TORQUE_PITCH_MAX)
        self.pid_roll = PID(kp=8.0, ki=0, kd=1.4,
                             output_min=-p.TORQUE_ROLL_MAX, output_max=p.TORQUE_ROLL_MAX)
        self.pid_yaw = PID(kp=2.0, ki=0, kd=0.5,
                            output_min=-p.TORQUE_YAW_MAX, output_max=p.TORQUE_YAW_MAX)

        self._yaw_to_force = p.K / p.B

    def set_target(self, x_target, y_target, z_target, psi_target=None, state=None):
        self.x_target = x_target
        self.y_target = y_target
        self.z_target = z_target
        if psi_target is not None:
            self.psi_target = psi_target
        self.reset_pids(state)

    def reset_pids(self, state=None):
        """Reset PID history. Seeding with the current error (instead of zero)
        avoids a derivative-kick spike when re-engaging with a large existing error."""
        if state is None:
            self.pid_z.reset()
            self.pid_x.reset()
            self.pid_y.reset()
            self.pid_pitch.reset()
            self.pid_roll.reset()
            self.pid_yaw.reset()
            return

        x, y, z = state["x"], state["y"], state["z"]
        psi = state["psi"]

        self.pid_z.reset(self.z_target - z)
        self.pid_x.reset(self.x_target - x)
        self.pid_y.reset(-(self.y_target - y))
        self.pid_pitch.reset(0.0)  # assume current tilt ~= desired tilt at reset time
        self.pid_roll.reset(0.0)
        self.pid_yaw.reset(wrap_angle(self.psi_target - psi))

    def _attitude_hold_and_mix(self, state, thrust_adjustment, theta_desired, phi_desired, dt):
        """Inner loop: track a commanded tilt/heading and mix into rotor speeds.

        Shared by full position-hold autopilot and manual (angle-mode) flight,
        so both always self-level and always hold altitude the same way.
        """
        phi, theta, psi = state["phi"], state["theta"], state["psi"]

        error_theta = wrap_angle(theta_desired - theta)
        error_phi = wrap_angle(phi_desired - phi)
        error_psi = wrap_angle(self.psi_target - psi)

        torque_pitch = self.pid_pitch.update(error_theta, dt)
        torque_roll = self.pid_roll.update(error_phi, dt)
        torque_yaw = self.pid_yaw.update(error_psi, dt)

        # tilt-compensated thrust: a tilted rotor loses vertical lift (~cos(angle)),
        # so scale up total thrust to keep the vertical component matching what
        # pid_z asked for, instead of losing altitude during aggressive maneuvers.
        tilt_cos = max(math.cos(phi) * math.cos(theta), 0.5)
        T = (p.MASS * p.GRAVITY + thrust_adjustment) / tilt_cos
        yaw_force = torque_yaw * self._yaw_to_force

        F_front = T / 4 - torque_pitch / (2 * p.L) + yaw_force / 4
        F_back = T / 4 + torque_pitch / (2 * p.L) + yaw_force / 4
        F_right = T / 4 - torque_roll / (2 * p.L) - yaw_force / 4
        F_left = T / 4 + torque_roll / (2 * p.L) - yaw_force / 4

        speeds = []
        for F in (F_front, F_right, F_back, F_left):
            F = max(F, 0.0)
            w = math.sqrt(F / p.K)
            w = min(max(w, p.W_MIN), p.W_MAX)
            speeds.append(w)

        return tuple(speeds)  # w_front, w_right, w_back, w_left

    def compute_motor_speeds(self, state, dt):
        """Full position-hold: fly to (x_target, y_target, z_target)."""
        x, y, z = state["x"], state["y"], state["z"]

        error_z = self.z_target - z
        error_x = self.x_target - x
        error_y = self.y_target - y

        thrust_adjustment = self.pid_z.update(error_z, dt)
        theta_desired = self.pid_x.update(error_x, dt)
        phi_desired = self.pid_y.update(-error_y, dt)

        return self._attitude_hold_and_mix(state, thrust_adjustment, theta_desired, phi_desired, dt)

    def compute_motor_speeds_manual(self, state, theta_desired, phi_desired, dt):
        """Angle-mode manual flight: hold altitude/heading, command tilt directly."""
        error_z = self.z_target - state["z"]
        thrust_adjustment = self.pid_z.update(error_z, dt)

        return self._attitude_hold_and_mix(state, thrust_adjustment, theta_desired, phi_desired, dt)
