class DCMotor:
    """
    DC Motor torque model (H-Bridge controlled)

    Equations:
        V_applied = Vmax * PWM * direction           # voltage applied to the motor
        E_back = Ke * w                              # back EMF proportional to angular speed
        I = (V_applied - E_back) / R                 # armature current from Ohm's law
        tau = Kt * I                                 # torque proportional to current
    """

    def __init__(
        self,
        torque_constant,  # Kt [Nm/A], torque per unit current
        back_emf_constant,  # Ke [V/(rad/s)], voltage induced per angular speed
        resistance,  # R [Ohm], motor winding resistance
        supply_voltage,  # Vmax [V], max voltage from H-bridge
    ):
        self.Kt = torque_constant
        self.Ke = back_emf_constant
        self.R = resistance
        self.Vmax = supply_voltage

    def step(self, dc_pwm: float, dc_direction: int, angular_speed):
        """
        Compute motor torque for the current timestep.

        Inputs:
            dc_pwm        : 0.0 - 1.0 (H-bridge PWM duty cycle)
            dc_direction  : +1 or -1 (H-bridge direction)
            angular_speed : w [rad/s], current rotor speed from physics

        Output:
            torque        : tau [Nm], output torque to apply to physics
        """

        # Clamp PWM and direction
        dc_pwm = max(0.0, min(1.0, dc_pwm))
        dc_direction = 1 if dc_direction >= 0 else -1

        # Applied voltage
        voltage = self.Vmax * dc_pwm * dc_direction  # V = Vmax * PWM * dir

        # Back EMF
        back_emf = self.Ke * angular_speed  # E = Ke * w

        # Algebraic current (no inductance)
        current = (voltage - back_emf) / self.R

        # Torque: tau = Kt * I
        torque = self.Kt * current
        return torque


class ServoMotor:
    """
    Servo motor torque generator using PD control.

    Equations:
        error = th_cmd - th                               # position error
        tau_cmd = Kp * error - Kd * w                     # PD control torque
        tau_cmd_saturated = clip(tau_cmd_cmd, -tau_cmd_max, tau_max)        # torque limit
        if |w| > servo_speed: tau_saturated = 0           # speed limiter
    """

    def __init__(
        self,
        max_torque,  # Nm, physical torque limit
        position_gain,  # Kp, proportional gain
        damping_gain,  # Kd, derivative gain
    ):
        self.max_torque = max_torque
        self.Kp = position_gain
        self.Kd = damping_gain

    def step(self, servo_angle, servo_speed, current_angle, angular_speed):
        """
        Compute servo torque for the current timestep.

        Inputs:
            servo_angle    : commanded angle th_cmd [rad]
            servo_speed    : max allowed speed [rad/s]
            current_angle  : actual angle th [rad] from physics
            angular_speed  : current speed w [rad/s] from physics

        Output:
            torque         : tau [Nm], output torque to apply to physics
        """

        #  Position error
        error = servo_angle - current_angle  # e = th_cmd - th

        # PD control torque
        torque = self.Kp * error - self.Kd * angular_speed  # tau = Kp*e - Kd*w

        # Speed limiter (optional)
        if abs(angular_speed) > servo_speed:
            torque = 0.0

        # Torque saturation
        torque = max(-self.max_torque, min(self.max_torque, torque))  # clip to tau_max
        return torque
