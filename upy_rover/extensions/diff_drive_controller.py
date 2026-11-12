from machine import Pin

from upy_rover.modules.regulated_wheel import RegulatedWheel


class DiffDriveController:
    def __init__(self, configs: dict = {"left_wheel_configs", "right_wheel_configs", "stby_id", "wheel_sep"}):
        self.left_wheel = RegulatedWheel(configs["left_wheel_configs"])
        self.right_wheel = RegulatedWheel(configs["right_wheel_configs"])
        self.stby_pin = Pin(configs["stby_id"], Pin.OUT)
        self.disable()
        # Variables
        self.ang_vel = 0.0
        self.lin_vel = 0.0
        # Constants
        self._whlsep = configs["wheel_sep"]
        
    def enable(self):
        self.stby_pin.value(1)
        
    def disable(self):
        self.stby_pin.value(0)
        
    def get_vels(self):
        self.ang_vel = (self.right_wheel.wheel.lin_vel - self.left_wheel.wheel.lin_vel) / self._whlsep
        self.lin_vel = (self.left_wheel.wheel.lin_vel + self.right_wheel.wheel.lin_vel) * 0.5
        
        return self.ang_vel, self.lin_vel
    
    def set_vels(self, ref_ang_vel, ref_lin_vel):
        ref_vel_left = ref_lin_vel - ref_ang_vel * self._whlsep * 0.5
        ref_vel_right = ref_lin_vel + ref_ang_vel * self._whlsep * 0.5
        self.left_wheel.set_vel(ref_vel_left)
        self.right_wheel.set_vel(ref_vel_right)
        
if __name__ == "__main__":
    # SETUP
    ddc = DiffDriveController(
        configs={
            "left_wheel_configs": {
                "motor_pin_ids": (15, 13 ,14),
                "sentient_wheel_configs": {
                    "encoder_ids": (18, 19),
                    "encoder_cpr": 28,
                    "wheel_radius": 0.021,
                    "gear_ratio": 100,
                    "vel_meas_freq": 150,
                },
                "K_P": 2.5,
                "K_I": 0.5,
                "K_D": 0.0,
                "reg_freq": 75,                    
            },
            "right_wheel_configs": {
                "motor_pin_ids": (9, 10 ,11),
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
            },
            "stby_id": 12,
            "wheel_sep": 0.116,
        }
    )
        
        
