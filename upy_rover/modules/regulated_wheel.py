from upy_rover.perception.sentient_wheel import SentientWheel
from upy_rover.action.motor import Motor
from machine import Timer


class RegulatedWheel:
    def __init__(
        self,
        motor_pin_ids: list | Tuple,
        encoder_pin_ids: list | tuple,
        mech_chars: dict,  # mechanical characteristics
        meas_freq: int = 200,
        pid_gains: dict = {
            "K_P": 0.0,
            "K_I": 0.0,
            "K_D": 0.0,
        },
        reg_freq: int = 50,
    ):
        self.motor = Motor(*motor_pin_ids)
        self.wheel = SentientWheel(encoder_pin_ids, mech_chars, meas_freq)
        self.vel_reg_timer = Timer(
            freq=reg_freq,
            mode=Timer.PERIODIC,
            callback=self.regulate_velocity,

        )
        # Constants
        self._kp = pid_gains["K_P"]
        self._ki = pid_gains["K_I"]
        self._kd = pid_gains["K_D"]
        # Variables
        self.ref_vel = 0.0
        self.prev_err = 0.0
        self.sum_err = 0.0


    def regulate_velocity(self, timer):
        err = self.ref_vel - self.wheel.lin_vel
        self.sum_err += err
        d_err = err - self.prev_err  # ignored rescaling by dt
        self.prev_err = err  # NOTE: error now is the past
        u = self._kp * err + self._ki * self.sum_err + self._kd * d_err
        if u > 0.0:
            if u > 1.0:
                u = 1.0
            self.motor.forward(u)
        elif u < 0.0:
            if u < -1.0:
                u = -1.0
            self.motor.backward(-u)
        else:
            self.motor.short_break()

    def set_vel(self, reference_value=0.0):
        self.sum_err = 0.0
        self.ref_vel = reference_value




