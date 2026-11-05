from encoder import Encoder
from machine import Timer
from math import pi


class SentientWheel(Encoder):
    def __init__(self, ch_a_id: int, ch_b_id: int):
        super().__init__(ch_a_id, ch_b_id)
        # Contants
        self.wheel_radius = 0.021  # m
        self.gear_ratio = 100  # 100:1
        self.cpr = 28
        # Velocity measurement timer
        self.vel_meas_timer = Timer(
            freq=150,
            mode=Timer.PERIODIC,
            callback=self.measure_velocity,
        )
        # Variables
        self.prev_counts = 0
        self.meas_ang_vel = 0.0
        self.meas_lin_vel = 0.0

    def measure_velocity(self, timer):
        curr_counts = self.counts
        delta_counts = curr_counts - self.prev_counts
        self.prev_counts = curr_counts  # UPDATE encoder counts
        counts_per_sec = delta_counts * self.meas_freq
        rev_per_sec_og = counts_per_sec / self.cpr  # rev/s
        ang_vel_og = rev_per_sec_og * 2 * pi  # rad/s
        self.meas_ang_vel = ang_vel_og / self.gear_ratio  # rad/s
        self.meas_lin_vel = self.meas_ang_vel * self.wheel_radius


# TEST
if __name__ == "__main__":
    from time import sleep

    # SETUP
    sw = SentientWheel(ch_a_id=16, ch_b_id=17)

    # LOOP
    while True:
        print(
            f"Measured angular velocity: {sw.meas_ang_vel}, linear velocity: {sw.meas_lin_vel}"
        )
        sleep(0.1)
