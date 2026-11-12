from machine import Pin


class Encoder:
    """A class to read a quadrature encoder using two GPIO pins."""

    def __init__(self, ch_a_id: int, ch_b_id: int):
        """Initialize the encoder with two GPIO pins for channels A and B.

        Args:
            ch_a_id (int): GPIO pin number for channel A.
            ch_b_id (int): GPIO pin number for channel B.
        """
        # GPIO pins setup
        self.ch_a = Pin(ch_a_id, Pin.IN)
        self.ch_b = Pin(ch_b_id, Pin.IN)
        self.ch_a.irq(
            trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=self.update_counts_a
        )
        self.ch_b.irq(
            trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=self.update_counts_b
        )
        # Variables
        self.counts = 0

    def update_counts_a(self, pin):
        val_a = pin.value()
        val_b = self.ch_b.value()
        if val_a == 0:  # Channel A Falling
            if val_b == 0:  # B leads A
                self.counts -= 1
            else:  # A leads B
                self.counts += 1
        else:  # Channel A Rising
            if val_b == 0:  # A leads B
                self.counts += 1
            else:  # B leads A
                self.counts -= 1

    def update_counts_b(self, pin):
        val_b = pin.value()
        val_a = self.ch_a.value()
        if val_b == 0:  # Channel B Falling
            if val_a == 0:  # A leads B
                self.counts += 1
            else:  # B leads A
                self.counts -= 1
        else:  # Channel B Rising
            if val_a == 0:  # B leads A
                self.counts -= 1
            else:  # A leads B
                self.counts += 1


if __name__ == "__main__":
    from time import sleep

    # SETUP
    enc = Encoder(ch_a_id=16, ch_b_id=17)

    # LOOP
    while True:
        print(f"Encoder counts: {enc.counts}")
        sleep(0.1)
