from machine import Pin

from upy_rover.extensions.regulated_wheel import RegulatedWheel


class DiffDriveController:
    def __init__(self, configs: dict):
        self.left_wheel = RegulatedWheel(configs["left_wheel_configs"])
        self.right_wheel = RegulatedWheel(configs["right_wheel_configs"])
        self.enable_pin = Pin(configs["enable_pin_id"], Pin.OUT)
        self.disable()
        # Constants
        self._whlsep = configs["wheel_sep"]
        # Variables
        self.ang_vel = 0.0
        self.lin_vel = 0.0

    def enable(self):
        self.enable_pin.value(1)

    def disable(self):
        self.enable_pin.value(0)

    def get_vels(self):
        self.ang_vel = (
            self.right_wheel.lin_vel - self.left_wheel.lin_vel
        ) / self._whlsep
        self.lin_vel = (self.left_wheel.lin_vel + self.right_wheel.lin_vel) * 0.5

        return self.ang_vel, self.lin_vel

    def set_vels(self, ref_ang_vel, ref_lin_vel):
        ref_vel_left = ref_lin_vel - ref_ang_vel * self._whlsep * 0.5
        ref_vel_right = ref_lin_vel + ref_ang_vel * self._whlsep * 0.5
        self.left_wheel.set_vel(ref_vel_left)
        self.right_wheel.set_vel(ref_vel_right)


if __name__ == "__main__":
    # SETUP
    diff_drive_controller_configs = {
        "left_wheel_configs": {
            "motor_pin_ids": (15, 13, 14),
            "sentient_wheel_configs": {
                "encoder_pin_ids": (18, 19),
                "encoder_cpr": 28,
                "gear_ratio": 100,
                "wheel_radius": 0.022,
                "meas_freq": 200,
            },
            "K_P": 2.0,
            "K_I": 0.8,
            "K_D": 0.2,
            "reg_freq": 100,
        },
        "right_wheel_configs": {
            "motor_pin_ids": (9, 11, 10),
            "sentient_wheel_configs": {
                "encoder_pin_ids": (16, 17),
                "encoder_cpr": 28,
                "gear_ratio": 100,
                "wheel_radius": 0.022,
                "meas_freq": 200,
            },
            "K_P": 2.0,
            "K_I": 0.8,
            "K_D": 0.2,
            "reg_freq": 100,
        },
        "enable_pin_id": 12,
        "wheel_sep": 0.116,
    }

    ddc = DiffDriveController(configs=diff_drive_controller_configs)
