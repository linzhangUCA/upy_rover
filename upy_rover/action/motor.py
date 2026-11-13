from machine import PWM, Pin


class Motor:
    """Base motor class drives a DC motor through an H-bridge driver.

    Manages direction via two digital GPIO pins and speed using a PWM pin.
    """

    def __init__(self, pwm_id: int, in1_id: int, in2_id: int, freq: int = 10_000):
        """Initializes the motor pins and configures the default PWM frequency.

        Args:
            pwm_id (int): GPIO pin number used for PWM speed control.
            in1_id (int): GPIO pin number for direction control line 1.
            in2_id (int): GPIO pin number for direction control line 2.
            freq (int, optional): PWM frequency in Hz. Defaults to 1000.
        """
        self.pwm_pin = PWM(Pin(pwm_id))
        self.pwm_pin.freq(freq)
        self.in1_pin = Pin(in1_id, Pin.OUT)
        self.in2_pin = Pin(in2_id, Pin.OUT)
        self.stop()

    def forward(self, effort: float = 0.0):
        """Drives the motor in the forward direction at the specified duty cycle.

        Args:
            effort (float, optional): Duty cycle normalized between 0.0 (off)
                and 1.0 (full speed). Defaults to 0.0.
        """
        if not 0.0 <= effort <= 1.0:
            raise ValueError(f"Effort must be between 0.0 and 1.0, got {effort}")
        dc_u16 = int(65_535 * effort)
        self.in1_pin.value(1)
        self.in2_pin.value(0)
        self.pwm_pin.duty_u16(dc_u16)

    def backward(self, effort: float = 0.0):
        """Drives the motor in the reverse direction at the specified duty cycle.

        Args:
            effort (float, optional): Duty cycle normalized between 0.0 (off)
                and 1.0 (full speed). Defaults to 0.0.
        """
        if not 0.0 <= effort <= 1.0:
            raise ValueError(f"Effort must be between 0.0 and 1.0, got {effort}")
        dc_u16 = int(65_535 * effort)
        self.in1_pin.value(0)
        self.in2_pin.value(1)
        self.pwm_pin.duty_u16(dc_u16)

    def short_brake(self):
        """Stops the motor using active braking.

        Sets both direction pins HIGH and PWM to LOW.
        """
        self.in1_pin.value(1)
        self.in2_pin.value(1)
        self.pwm_pin.duty_u16(0)

    def stop(self):
        """Stops the motor using passive coasting.

        Sets both direction pins LOW and PWM to HIGH.
        """
        self.in1_pin.value(0)
        self.in2_pin.value(0)
        self.pwm_pin.duty_u16(65_535)


if __name__ == "__main__":
    from time import sleep

    # SETUP
    STBY = Pin(12, Pin.OUT)
    STBY.value(1)
    print("Motor driver enabled.")
    mot = Motor(pwm_id=9, in1_id=11, in2_id=10)  # right
    # mot = Motor(pwm_id=15, in1_id=13, in2_id=14)  # left
    dc = 0.0
    inc = 1 / 200

    # LOOP
    for _ in range(200):
        dc += inc
        mot.forward(dc)
        print(f"Motor forward at {dc:.2%} duty cycle.")
        sleep(4 / 200)
    for _ in range(200):
        dc -= inc
        mot.forward(dc)
        print(f"Motor forward at {dc:.2%} duty cycle.")
        sleep(4 / 200)
    for _ in range(200):
        dc += inc
        mot.backward(dc)
        print(f"Motor backward at {dc:.2%} duty cycle.")
        sleep(4 / 200)
    for _ in range(200):
        dc -= inc
        mot.backward(dc)
        print(f"Motor backward at {dc:.2%} duty cycle.")
        sleep(4 / 200)

    mot.stop()
    print("Motor coasting.")
    sleep(0.5)
    mot.short_brake()
    print("Motor braking.")
    sleep(0.5)
    STBY.value(0)
    print("Motor driver disabled.")
