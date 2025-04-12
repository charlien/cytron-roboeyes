import asyncio
import servo # type: ignore
from gamepad import GamePad
from radar import RadarSensor
from neopixel import NeoPixel
from adafruit_simplemath import map_unconstrained_range, map_range
from statuslight import StatusLight
class EyeBall:
    def __init__(self, 
                 servo_x: servo.Servo, servo_y: servo.Servo,
                 start_angle_x: int, stop_angle_x: int,
                 start_angle_y: int, stop_angle_y: int,
                 default_angle_xy: tuple[int, int] = (88, 67),
                 debug: bool = False, 
                 status_light: StatusLight = None) -> None:
        
        self.servo_x = servo_x
        self.servo_y = servo_y

        self.start_angle_x = start_angle_x
        self.stop_angle_x = stop_angle_x

        self.start_angle_y = start_angle_y
        self.stop_angle_y = stop_angle_y

        self.last_angle_x = None
        self.last_angle_y = None
        self.default_angle_xy = default_angle_xy

        self.status_light = status_light
        self.changed = False
        self.debug = debug

    def __str__(self):
        return f"EyeBall(servo_x={self.servo_x}, servo_y={self.servo_y})"
    
    def __repr__(self):
        return self.__str__()
    
    def status(self):
        return f"eyeball: {self.last_angle_x}, {self.last_angle_y}"

    def get_default_angle_xy(self):
        return self.default_angle_xy

    async def update(self, pad: GamePad, radar: RadarSensor):
        angle_x, angle_y = self.get_default_angle_xy()

        if pad.button_start.value:
            # Move the eyeballs to the closest target
            if not radar.running and not radar.failed:
                asyncio.create_task(radar.run(debug=True))
            else:
                if radar.closest:
                    closest = radar.closest
                    angle_x = map_range(closest['angle'], *radar.angle_range, self.start_angle_x, self.stop_angle_x)
                    # Radar doesn't provide an angle for the y axis, so we use the gamepad
                    angle_y = await pad.get_angle_y(self.start_angle_y, self.stop_angle_y)
        else:
            if radar.running: radar.stop()
            # Move the eyeballs with the gamepad
            angle_x = await pad.get_angle_x(self.start_angle_x, self.stop_angle_x)
            angle_y = await pad.get_angle_y(self.start_angle_y, self.stop_angle_y)
        
        angle_x, angle_y = int(angle_x), int(angle_y)
        # Move the eyeballs if they have changed
        if angle_x != self.last_angle_x or angle_y != self.last_angle_y:
            self.move_direct(angle_x, angle_y)
            self.changed = True
        else:
            self.changed = False

    def has_changed(self):
        return self.changed

    def _is_valid_angle(self, angle):
        return angle >= 0 and angle <= 180

    def move_direct(self, angle_x, angle_y):
        if self._is_valid_angle(angle_x) and self._is_valid_angle(angle_y):
            
            self.servo_x.angle = angle_x
            self.last_angle_x = angle_x

            self.servo_y.angle = angle_y
            self.last_angle_y = angle_y

