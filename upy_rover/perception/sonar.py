from time import ticks_us

from machine import PWM, Pin


class Sonar:
    """A class for HC-SR04 ultrasonic distance sensor."""

    def __init__(self, trig_id: int, echo_id: int, trig_freq: int = 12):
        """Initializes the distance sensor with specified GPIO pins and trigger frequency.
        Args:
            trig_id (int): GPIO pin number for the trigger signal.
            echo_id (int): GPIO pin number for the echo signal.
            trig_freq (int, optional): Frequency of the trigger signal in Hz. Defaults to 12 Hz.
        """
        # GPIO pin setup
        self.trig_pin = PWM(Pin(trig_id), freq=trig_freq, duty_ns=10_000)
        self.echo_pin = Pin(echo_id, Pin.IN, Pin.PULL_DOWN)
        self.echo_pin.irq(
            trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=self.calc_dist
        )
        # Variables
        self.distance = None
        self.echo_up_instant = None

    def calc_dist(self, pin):
        if pin.value():  # echo pin pulled up
            self.echo_up_instant = ticks_us()
        else:  # echo pulled down
            dt = ticks_us() - self.echo_up_instant
            if dt < 100:  # too short, may be a mis-detection
                self.distance = 0.0
            elif 100 <= dt < 38000:
                self.distance = 343 * dt * 1e-6 * 0.5  # v(m/s) * t (s) / 2
            else:  # out of range
                self.distance = None


if __name__ == "__main__":
    from utime import sleep_ms

    # SETUP
    sonar = Sonar(trig_id=3, echo_id=2, trig_freq=15)

    # LOOP
    while True:
        print(f"Distance: {sonar.distance} m")
        sleep_ms(200)
