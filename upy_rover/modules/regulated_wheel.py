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
        reg_freq: int = 100,
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
        self._reg_freq = reg_freq
        # Variables
        self.ref_vel = 0.0
        self.prev_err = 0.0
        self.sum_err = 0.0
        self.reg_vel_counter = 0

    def regulate_velocity(self, timer):
        if self.ref_vel == 0.0 or self.reg_vel_counter > self._reg_freq:
            self.motor.short_break()
            self.prev_err = 0.0
        else:
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
            self.reg_vel_counter += 1


    def set_vel(self, reference_value=0.0):
        if reference_value is not self.ref_vel:
            self.ref_vel = reference_value
            self.prev_err = 0.0
            self.sum_err = 0.0
            self.reg_vel_counter = 0

if __name__ == "__main__":
    from time import sleep
    from machine import Pin
    # SETUP
    regw = RegulatedWheel(
        motor_pin_ids=(9, 10, 11),
        encoder_pin_ids=(16, 17),
        mech_chars={
            "wheel_radius": 0.021,  # m
            "gear_ratio": 100,
            "encoder_cpr": 28,
        },
        meas_freq=150,
        pid_gains={
            "K_P": 2.0,
            "K_I": 0.5,
            "K_D": 0.0,
        },
        reg_freq=75,
    )
    STBY = Pin(12, Pin.OUT)
    STBY.value(1)  # enable motor driver

    # LOOP
    for refv in (-0.3, -0.2, -0.1, 0.0, 0.2, 0.4):
        regw.set_vel(refv)
        for _ in range(20):  # 2 sec for each ref vel
            print(f"Reference velocity: {refv}, measured velocity: {regw.wheel.lin_vel}")
            sleep(0.1)

    STBY.value(0)  # disable motor driver

