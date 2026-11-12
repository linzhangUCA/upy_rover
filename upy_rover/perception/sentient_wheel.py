from upy_rover.perception.encoder import Encoder
from machine import Timer
from math import pi


class SentientWheel(Encoder):
    def __init__(
        self,
        encoder_pin_ids: list | tuple,
        mech_chars: dict,  # mechanical characteristics
        meas_freq: int = 200,
    ):
        super().__init__(*encoder_pin_ids)
        # Contants
        self._whlrad = mech_chars["wheel_radius"]
        self._grat = mech_chars["gear_ratio"]
        self._cpr = mech_chars["encoder_cpr"]
        self._meas_freq = meas_freq
        # Velocity measurement timer
        self.vel_meas_timer = Timer(
            freq=meas_freq,  # Hz
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
        cps = delta_counts * self._meas_freq  # encoder counts per second
        rps_og = cps / self._cpr  # original shaft's revs per second
        ang_vel_og = rps_og * 2 * pi  # original shaft's ang. vel., rad/s
        self.ang_vel = ang_vel_og / self._grat  # rad/s
        self.lin_vel = self.ang_vel * self._whlrad  # m/s


# TEST
if __name__ == "__main__":
    from time import sleep

    # SETUP
    sw = SentientWheel(
        encoder_pin_ids=(16, 17),
            mech_chars={
                "wheel_radius": 0.021,  # m
                "gear_ratio": 100,
                "encoder_cpr": 28,
            },
        meas_freq=100,
    )


    # LOOP
    while True:
        print(
            f"Measured angular velocity: {sw.ang_vel}, linear velocity: {sw.lin_vel}"
        )
        sleep(0.1)
