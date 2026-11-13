from math import pi

from machine import Timer

from upy_rover.perception.encoder import Encoder


class SentientWheel(Encoder):
    def __init__(self, configs: dict):
        super().__init__(*configs["encoder_pin_ids"])
        # Contants
        self._cpr = configs["encoder_cpr"]
        self._gratio = configs["gear_ratio"]
        self._whlrad = configs["wheel_radius"]
        self._meas_freq = configs["vel_meas_freq"]
        # Variables
        self.prev_counts = 0
        self.ang_vel = 0.0
        self.lin_vel = 0.0
        # Velocity measurement timer
        self.vel_meas_timer = Timer(
            freq=self._meas_freq,  # Hz
            mode=Timer.PERIODIC,
            callback=self.measure_velocity,
        )

    def measure_velocity(self, timer):
        curr_counts = self.counts
        delta_counts = curr_counts - self.prev_counts
        self.prev_counts = curr_counts  # NOTE: counts now is the past
        cps = delta_counts * self._meas_freq  # encoder counts per second
        rps_og = cps / self._cpr  # original shaft's revs per second
        ang_vel_og = rps_og * 2 * pi  # original shaft's ang. vel., rad/s
        self.ang_vel = ang_vel_og / self._gratio  # rad/s
        self.lin_vel = self.ang_vel * self._whlrad  # m/s


# TEST
if __name__ == "__main__":
    from time import sleep
    from machine import Pin
    from upy_rover.action.motor import Motor

    # SETUP
    left_configs = {
        "encoder_pin_ids": (18, 19),
        "encoder_cpr": 28,
        "gear_ratio": 100,
        "wheel_radius": 0.022,
        "vel_meas_freq": 200,
    }
    right_configs = {
        "encoder_pin_ids": (16, 17),
        "encoder_cpr": 28,
        "gear_ratio": 100,
        "wheel_radius": 0.022,
        "vel_meas_freq": 200,
    }
    sw = SentientWheel(configs=left_configs)
    m = Motor(15, 13, 14)
    # sw = SentientWheel(configs=right_configs)
    # m = Motor(9, 10, 11)
    STBY = Pin(12, Pin.OUT)
    STBY.value(1)

    # LOOP
    efforts = (0, 0.25, 0.5, 0.75, 1.0)
    for ef in efforts:
        m.forward(ef)
        for _ in range(20):
            print(f"Measured linear velocity: {sw.lin_vel}")
            sleep(0.1)
    for ef in reversed(efforts):
        m.forward(ef)
        for _ in range(20):
            print(f"Measured linear velocity: {sw.lin_vel}")
            sleep(0.1)

    STBY.value(0)

    # while True:
    #     print(f"Measured angular velocity: {sw.ang_vel}, linear velocity: {sw.lin_vel}")
    #     sleep(0.1)
