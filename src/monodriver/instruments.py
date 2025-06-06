import abstract
import yaml
from typing import Literal
import serial
import time

class ArduinoSerialManager():
    def __init__(self, port: str = "/dev/ttyACM0", baudrate: int = 9600, timeout: int = 2):
        self._ser = serial.Serial(port, baudrate, timeout=timeout)

    def send_command(self, command: str):
        self._ser.write(command.encode())
        response = self._ser.readline().decode().strip()
        while response == "":
            time.sleep(0.1)
            response = self._ser.readline().decode().strip()
        return response

    def close(self):
        self._ser.close()


class ArduinoTTL(abstract.TTL):
    def __init__(self, ser: ArduinoSerialManager, pin: int, state: bool = False):
        self._pin = pin
        self._ser = ser

    def _get_state(self):
        return super()._get_state()

    def set_state(self, state: bool):
        command = f"{self._pin} {int(state)}\n"
        response = self._ser.send_command(command)
        if response.startswith("ERR"):
            raise ValueError(f"Arduino error: {response}")
        return response
    
    def pulse(self, ontime, offtime, amount):
        command = f"{self._pin} {amount} {ontime} {offtime}"
        print(command)
        response = self._ser.send_command(command)
        if response.startswith("ERR"):
            raise ValueError(f"Arduino error: {response}")
        return response


class DRV8825(abstract.MotorDriver):
    ttls: dict
    _MODES = (
        (False, False, False),  # Full step
        (True, False, False),  # Half
        (False, True, False),  # Quarter
        (True, True, False),  # Eighth
        (True, True, True),  # Sixteenth
    )

    def __init__(self, ttls: dict, stepping: int = 0):
        for ttl in ttls:
            setattr(self, ttl, ttls[ttl])
        self._stepping = stepping
        self.set_stepping(self._stepping)

    def set_stepping(self, stepping: int):
        m1, m2, m3 = self._MODES[stepping]
        try:
            self.m1.set_state(m1)
            self.m2.set_state(m2)
            self.m3.set_state(m3)
        except AttributeError:
            # TODO: add some kind of warning that this is the default.
            self._stepping = 0

    def get_stepping(self):
        try:
            self._stepping = self._MODES.index(
                (self.m1.state, self.m2.state, self.m3.state)
            )
            return self._stepping
        except AttributeError:
            return self._stepping

    # Obs: Minimum pulse duration is 1 micro second.
    # Obs2: slope has to be rising for step pin to work.
    def step(self, ontime=1, offtime=1, amount=1):
        self.pin_step.pulse(ontime, offtime, amount)

class M061CS02(abstract.Motor):
    _STEPS_MODE = (200, 400, 800, 1600, 3200)

    def __init__(self, driver, steps: int = 400, angle: float = 0.0):
        self._driver = driver
        self._angle = angle
        self._angle_relative = angle % 360
        self.steps = steps
        self._min_angle = 360.0 / self.steps
        self._min_offtime = 5
        self._min_ontime = 5

    def rotate(self, angle: float):
        relative_angle = angle - self._angle
        return self.rotate_relative(relative_angle)

    def rotate_relative(self, angle: float, change_angle: bool = True):
        cw, angle = self._cw_from_angle(angle), abs(angle)
        self._driver.direction.set_state(cw)
        steps = self._steps_from_angle(angle)
        self.rotate_step(steps, cw, change_angle=change_angle)
        angle_rotated = self._angle_sign(cw) * self._angle_from_steps(steps)
        return angle_rotated

    def rotate_step(self, steps: int, cw: bool, change_angle: bool = True):
        self._driver.direction.set_state(cw)
        self._driver.step(
            ontime=self._min_ontime, offtime=self._min_offtime, amount=steps
        )
        if change_angle:
            angle_change_sign = int(cw) * 2 - 1
            self._angle = round(
                self._angle + angle_change_sign * self.min_angle * steps, 5
            )

    def _angle_sign(self, cw: bool):
        return 2 * cw - 1

    def _angle_from_steps(self, steps: int):
        if steps >= 0:
            return self.min_angle * steps
        else:
            raise ValueError(f"steps should be greater than 0, not {steps}.")

    def _steps_from_angle(self, angle: float):
        if angle >= 0:
            return int(angle / self.min_angle)
        else:
            raise ValueError(f"angle should be greater than 0, not {angle}.")

    def _cw_from_angle(self, angle: float):
        return angle > 0

    @property
    def angle(self):
        return self._angle

    @property
    def angle_relative(self):
        return self._angle % 360

    @property
    def min_angle(self):
        return 360 / self.steps

    @property
    def steps(self):
        return self._STEPS_MODE[self._driver.get_stepping()]

    @steps.setter
    def steps(self, steps: int = 200):
        self._driver.set_stepping(self._STEPS_MODE.index(steps))

    def set_origin(self, angle: float = 0):
        self._angle = angle

class Monochromator:
    def __init__(
        self,
        motor: abstract.Motor,
        limit_switch: ArduinoTTL | None = None,#: rpp.digital.RPDI,
        # TODO: improve path handling
        calibration_path: str = None,
    ):
        # TODO: move this to calibration.
        self.CALIB_ATTRS = [
            "_wl_step_ratio",
            "_greater_wl_cw",
            "_max_wl",
            "_min_wl",
            "_home_wavelength",
        ]
        self._motor = motor
        if limit_switch is not None:
            self._limit_switch = limit_switch
        if calibration_path is not None:
            self.load_calibration(calibration_path)

    @classmethod
    def constructor_default(
        cls,
        pin_step: tuple[Literal["n", "p"], int],
        pin_direction: tuple[Literal["n", "p"], int],
        limit_switch: tuple[Literal["n", "p"], int],
        MOTOR_DRIVER: abstract.MotorDriver = DRV8825,
        MOTOR: abstract.Motor = M061CS02,
        calibration_path: str = None,
    ):
        ttls = {
            "pin_step": rpp.digital.RPDO(pin_step, state=False),
            "direction": rpp.digital.RPDO(pin_direction, state=True),
        }

        driver = MOTOR_DRIVER(ttls)
        motor = MOTOR(driver)
        limit_switch = rpp.digital.RPDI(pin=limit_switch)
        return cls(motor, limit_switch=limit_switch, calibration_path=calibration_path)

    @property
    def wavelength(self):
        try:
            return self._wavelength
        except:
            return None

    @property
    def min_wl(self):
        return self._min_wl

    @property
    def max_wl(self):
        return self._max_wl

    @property
    def greater_wl_cw(self):
        return self._greater_wl_cw

    @property
    def wl_step_ratio(self):
        return self._wl_step_ratio

    @property
    def home_wavelength(self):
        return self._home_wavelength

    def set_wavelength(self, wavelength: float):
        if self.check_safety(wavelength):
            self._wavelength = wavelength
        else:
            print(f"Wavelength must be between {self._min_wl} and {self._max_wl}")

    def check_safety(self, wavelength):
        return self._min_wl <= wavelength <= self._max_wl

    def goto_wavelength(self, wavelength: float):
        if self.check_safety(wavelength):
            steps = self._steps_from_wl(wavelength)
            cw = self._cw_from_wl(wavelength)
            self._motor.rotate_step(steps, cw)
            self._wavelength = wavelength
        else:
            print(f"Wavelength must be between {self._min_wl} and {self._max_wl}")
        return self._wavelength

    def _steps_from_wl(self, wavelength: float):
        return abs(int((wavelength - self.wavelength) / self._wl_step_ratio))

    def _cw_from_wl(self, wavelength: float):
        cw = (wavelength - self.wavelength) > 0
        cw = not (cw ^ self.greater_wl_cw)
        return cw

    @property
    def limit_switch(self):
        return self._limit_switch

    def load_calibration(self, path: str):  # wavelength
        # TODO: make a better implementation of calibration loading.
        with open(path, "r") as f:
            self._calibration = yaml.full_load(f)
        for param in self._calibration:
            setattr(self, f"_{param}", self._calibration[param])
        calibration_complete = True
        for param in self.CALIB_ATTRS:
            if not hasattr(self, param):
                calibration_complete = False
                print(f"Calibration parameter {param[1:]} is missing.")
        if not calibration_complete:
            print("Calibration is incomplete.")

    def calibrate(self):
        ui.SpectrometerCalibrationInterface(self).cmdloop()
        # TODO: make a better implementation of calibration loading.
        self.load_calibration(self.calibration_path)

    def swipe_wavelengths(
        self,
        starting_wavelength: float = None,
        ending_wavelength: float = None,
        wavelength_step: float = None,
    ):
        if starting_wavelength is None:
            starting_wavelength = self.min_wl
        if ending_wavelength is None:
            ending_wavelength = self.max_wl
        if wavelength_step is None:
            wavelength_step = abs(self.wl_step_ratio)

        n_measurements = int(
            (ending_wavelength - starting_wavelength) / wavelength_step
        )
        for i in range(n_measurements):
            yield self.goto_wavelength(starting_wavelength + i * wavelength_step)

    def home(self, set_wavelength=True, ignore_limit=False):
        steps_done = 0
        # This ensures ending wavelength is above 0.
        if ignore_limit:
            print("WARNING: IGNORING LIMIT SWITCH SAFETY LIMIT")
            steps_limit = 2e3
        else:
            steps_limit = abs(self.home_wavelength / self.wl_step_ratio)
        # TODO: if you start at the home_wavelength, this doesn't work
        while self.limit_switch.state and steps_done < steps_limit:
            self._motor.rotate_step(1, not self._greater_wl_cw)
            steps_done += 1
        if set_wavelength and steps_done < steps_limit:
            self.set_wavelength(self.home_wavelength)
        elif steps_done >= steps_limit:
            print("Danger warning:")
            print(
                f"Wavelength could not be set. Call home method again if and only if wavelength is greater than {self.home_wavelength}"
            )