from upy_rover.perception.sentient_wheel import SentientWheel
from upy_rover.action.motor import Motor
from machine import Timer


class RegulatedWheel:
    def __init__(
        self,
        configs: dict = {
            "motor_pin_ids",  # tuple
            "sentient_wheel_configs",  # dict
            "K_P",
            "K_I",
            "K_D",
            "reg_freq",
        }
    ):
        # Base components
        self.motor = Motor(*configs["motor_pin_ids"])
        self.wheel = SentientWheel(configs["sentient_wheel_configs"])
        # Constants
        self._kp = configs["K_P"]
        self._ki = configs["K_I"]
        self._kd = configs["K_D"]
        self._freq = configs["reg_freq"]
        # Controller timer
        self.vel_reg_timer = Timer(
            freq=self._freq,
            mode=Timer.PERIODIC,
            callback=self.regulate_velocity,
        )
        # Variables
        self.ref_vel = 0.0
        self.prev_err = 0.0
        self.sum_err = 0.0
        self.reg_counter = 0

    def regulate_velocity(self, timer):
        if self.ref_vel == 0.0 or self.reg_counter > self._freq:
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
            self.reg_counter += 1


    def set_vel(self, reference_value=0.0):
        if reference_value is not self.ref_vel:
            self.ref_vel = reference_value
            self.prev_err = 0.0
            self.sum_err = 0.0
            self.reg_counter = 0

if __name__ == "__main__":
    from time import sleep
    from machine import Pin
    # SETUP
    regw = RegulatedWheel(
        configs={
            "motor_pin_ids": (9, 10, 11),
            "sentient_wheel_configs": {
                "encoder_ids": (16, 17),
                "encoder_cpr": 28,
                "wheel_radius": 0.021,
                "gear_ratio": 100,
                "vel_meas_freq": 150,
            },
            "K_P": 2.5,
            "K_I": 0.5,
            "K_D": 0.0,
            "reg_freq": 75,
        }
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

