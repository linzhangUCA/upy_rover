from upy_rover.perception.encoder import Encoder
from machine import Timer
from math import pi


class SentientWheel(Encoder):
    def __init__(
        self,
        configs: dict = {
            "encoder_ids",
            "encoder_cpr",
            "wheel_radius",
            "gear_ratio",
            "vel_meas_freq",
        },
    ):
        super().__init__(*configs["encoder_ids"])
        # Contants
        self._whlrad = configs["wheel_radius"]
        self._grat = configs["gear_ratio"]
        self._cpr = configs["encoder_cpr"]
        self._freq = configs["vel_meas_freq"]
        # Velocity measurement timer
        self.vel_meas_timer = Timer(
            freq=self._freq,  # Hz
            mode=Timer.PERIODIC,
            callback=self.measure_velocity,
        )
        # Variables
        self.prev_counts = 0
        self.ang_vel = 0.0
        self.lin_vel = 0.0

    def measure_velocity(self, timer):
        curr_counts = self.counts
        delta_counts = curr_counts - self.prev_counts
        self.prev_counts = curr_counts  # NOTE: update previous counts
        cps = delta_counts * self._freq  # encoder counts per second
        rps_og = cps / self._cpr  # original shaft's revs per second
        ang_vel_og = rps_og * 2 * pi  # original shaft's ang. vel., rad/s
        self.ang_vel = ang_vel_og / self._grat  # rad/s
        self.lin_vel = self.ang_vel * self._whlrad  # m/s


# TEST
if __name__ == "__main__":
    from time import sleep

    # SETUP
    sw = SentientWheel(
        configs={
            "encoder_ids": (16, 17),
            "encoder_cpr": 28,
            "wheel_radius": 0.021,
            "gear_ratio": 100,
            "vel_meas_freq": 150,
        }
    )


    # LOOP
    while True:
        print(
            f"Measured angular velocity: {sw.ang_vel}, linear velocity: {sw.lin_vel}"
        )
        sleep(0.1)
