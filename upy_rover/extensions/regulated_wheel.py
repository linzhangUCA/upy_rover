from machine import Timer

from upy_rover.action.motor import Motor
from upy_rover.extensions.sentient_wheel import SentientWheel


def clamp(x, min_val=-1.0, max_val=1.0):
    return max(min_val, min(x, max_val))


class RegulatedWheel(Motor, SentientWheel):
    def __init__(self, configs: dict):
        Motor.__init__(self, *configs["motor_pin_ids"])
        SentientWheel.__init__(self, configs["sentient_wheel_configs"])
        # Constants
        self._kp = configs["K_P"]
        self._ki = configs["K_I"]
        self._kd = configs["K_D"]
        self._reg_freq = configs["reg_freq"]
        # Variables
        self.prev_err = 0.0
        self.sum_err = 0.0
        self.reg_counter = 0
        self.ref_vel = 0.0  # reference linear velocity
        # self.control_signal = 0.0  # in terms of pwm duty cycle
        # Controller timer
        self.vel_reg_timer = Timer(
            freq=self._reg_freq,
            mode=Timer.PERIODIC,
            callback=self.regulate_velocity,
        )

    def regulate_velocity(self, timer):
        if self.ref_vel == 0.0 or self.reg_counter > self._reg_freq:
            self.short_brake()
            self.prev_err = 0.0
        else:
            err = self.ref_vel - self.lin_vel
            self.sum_err += err
            d_err = err - self.prev_err  # ignored rescaling by dt
            self.prev_err = err  # NOTE: error now is the past
            u = clamp(self._kp * err + self._ki * self.sum_err + self._kd * d_err)
            if u > 0.0:
                self.forward(u)
            elif u < 0.0:
                self.backward(-u)
            else:
                self.short_brake()
            self.reg_counter += 1

    def set_vel(self, reference_value=0.0):
        if reference_value is not self.ref_vel:
            self.prev_err = 0.0
            self.sum_err = 0.0
        self.ref_vel = reference_value
        self.reg_counter = 0


if __name__ == "__main__":
    from time import sleep
    from machine import Pin

    # SETUP
    left_configs = {
        "motor_pin_ids": (15, 13, 14),
        "sentient_wheel_configs": {
            "encoder_pin_ids": (18, 19),
            "encoder_cpr": 28,
            "gear_ratio": 100,
            "wheel_radius": 0.022,
            "meas_freq": 200,
        },
        "K_P": 2.0,
        "K_I": 0.8,
        "K_D": 0.2,
        "reg_freq": 100,
    }

    right_configs = {
        "motor_pin_ids": (9, 11, 10),
        "sentient_wheel_configs": {
            "encoder_pin_ids": (16, 17),
            "encoder_cpr": 28,
            "gear_ratio": 100,
            "wheel_radius": 0.022,
            "meas_freq": 200,
        },
        "K_P": 2.0,
        "K_I": 0.8,
        "K_D": 0.2,
        "reg_freq": 100,
    }

    regw = RegulatedWheel(configs=left_configs)
    STBY = Pin(12, Pin.OUT)
    STBY.value(1)  # enable motor driver

    # LOOP
    for refv in (0.1, 0.2, 0.3, 0.4, 0.0):
        regw.set_vel(refv)
        for _ in range(20):  # 2 sec for each ref vel
            print(f"Reference velocity: {refv}, measured velocity: {regw.lin_vel}")
            sleep(0.1)

    STBY.value(0)  # disable motor driver
