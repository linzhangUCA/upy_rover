# ruff: noqa
# pyright: reportUndefinedVariable=false, reportCallIssue=false, reportArgumentType=false, reportAttributeAccessIssue=false
import rp2
from machine import Pin, Timer


@rp2.asm_pio()
def count_edges():
    """ """
    set(x, 0x00000000)  # init counter
    wrap_target()

    label("falling")
    wait(0, pin, 0)
    jmp(pin, "base_fall_bud_fall")  # buddy channel is still HIGH

    label("bud_fall_base_fall")  # e.g. BA
    jmp(x_dec, "fall_dec")
    label("fall_dec")
    jmp("push_fall_counts")

    label("base_fall_bud_fall")  # e.g. AB
    mov(x, invert(x))  # X = ~X
    jmp(x_dec, "fall_inc")  # X = ~X - 1
    label("fall_inc")
    mov(x, invert(x))  # X = ~(~X - 1) = X + 1

    label("push_fall_counts")
    mov(isr, x)
    push(noblock)  # send counts to RX FIFO

    label("rising")
    wait(1, pin, 0)
    jmp(pin, "bud_rise_base_rise")  # B is already HIGH

    label("base_rise_bud_rise")  # AB
    mov(x, invert(x))
    jmp(x_dec, "rise_inc")
    label("rise_inc")
    mov(x, invert(x))
    jmp("push_rise_counts")

    label("bud_rise_base_rise")  # BA
    jmp(x_dec, "rise_dec")
    label("rise_dec")

    label("push_rise_counts")
    mov(isr, x)
    push(noblock)  # send counts to RX FIFO

    wrap()


class PIOEncoder:
    """A class to read a quadrature encoder using two GPIO pins. An implementation of RP2's PIO"""

    base_sm_id = 0  # StateMachie index for counting edges on channel A.

    def __init__(self, ch_a_id: int, ch_b_id: int):
        """Initialize the encoder with two GPIO pins for channels A and B.

        Args:
            ch_a_id (int): GPIO pin number for channel A.
            ch_b_id (int): GPIO pin number for channel B.
        """
        # Config encoder pins
        self.ch_a = Pin(ch_a_id, Pin.IN)
        self.ch_b = Pin(ch_b_id, Pin.IN)
        # Config state machines to read high-speed signals
        self.sm_a = rp2.StateMachine(
            PIOEncoder.base_sm_id,
            count_edges,
            freq=150_000_000,
            in_base=self.ch_a,
            jmp_pin=self.ch_b,
        )  # counter state machine based on channel A
        self.sm_a.active(1)
        print(f"State machine {PIOEncoder.base_sm_id} activated.")
        PIOEncoder.base_sm_id += 1
        self.sm_b = rp2.StateMachine(
            PIOEncoder.base_sm_id,
            count_edges,
            freq=150_000_000,
            in_base=self.ch_b,
            jmp_pin=self.ch_a,
        )  # counter state machine based on channel B
        self.sm_b.active(1)
        print(f"State machine {PIOEncoder.base_sm_id} activated.")
        PIOEncoder.base_sm_id += 2
        # Config encoder counts collection timer
        self.count_timer = Timer(
            freq=200,
            mode=Timer.PERIODIC,
            callback=self.get_counts,
        )
        # Variables
        self._counts_a = 0
        self._counts_b = 0
        self.counts = 0

    def get_counts(self, timer):
        while self.sm_a.rx_fifo():
            self._counts_a = self.sm_a.get()
            if (
                self._counts_a >= 1 << 31
            ):  # convert uint32 to int32 if counts_a >= 2**31
                self._counts_a -= 1 << 32  # counts_a -= 2**32
        while self.sm_b.rx_fifo():
            self._counts_b = self.sm_b.get()
            if (
                self._counts_b >= 1 << 31
            ):  # convert uint32 to int32 if counts_b >= 2**31
                self._counts_b -= 1 << 32  # counts_b -= 2**32

        self.counts = self._counts_a - self._counts_b  # counts b has reversed logic


if __name__ == "__main__":
    from time import sleep

    # SETUP
    enc = PIOEncoder(16, 17)

    # LOOP
    while True:
        print(f"Encoder counts: {enc.counts}")
        sleep(0.1)
