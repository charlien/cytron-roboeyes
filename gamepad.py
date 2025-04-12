from eye_angle_provider import EyeAngleProvider
from micropython import const # type: ignore
import asyncio

from adafruit_seesaw.seesaw import Seesaw
from adafruit_debouncer import Debouncer, Button
from adafruit_simplemath import map_range

BUTTON_X = const(6)
BUTTON_Y = const(2)
BUTTON_A = const(5)
BUTTON_B = const(1)
BUTTON_SELECT = const(0)
BUTTON_START = const(16)
JOY_X = const(14)
JOY_Y = const(15)

BUTTON_MASK = const(
    (1 << BUTTON_X)
    | (1 << BUTTON_Y)
    | (1 << BUTTON_A)
    | (1 << BUTTON_B)
    | (1 << BUTTON_SELECT)
    | (1 << BUTTON_START)
)

class GameButton:
    def __init__(self, name, default_value = False, is_toggle = False, short_duration_ms=200, long_duration_ms=500):
        self._value = default_value
        self._name = name
        self._short_duration_ms = short_duration_ms
        self._long_duration_ms = long_duration_ms
        self._button = Button(lambda: self._value, self._short_duration_ms, self._long_duration_ms)
        self._is_toggle = is_toggle
        self._toggle_state = False

    @property
    def value(self):
        return self._toggle_state if self._is_toggle else self._button.pressed
    
    @value.setter
    def value(self, value):

        self._value = value
        self._button.update()
        
        if self._is_toggle and self._button.pressed:
            self._toggle_state = not self._toggle_state
        
    @property
    def name(self):
        return self._name
    
    @name.setter
    def name(self, value):
        self._name = value
        
    @property
    def short_duration_ms(self):
        return self._short_duration_ms
    
    @short_duration_ms.setter
    def short_duration_ms(self, value):
        self._short_duration_ms = value
        
    @property
    def long_duration_ms(self):
        return self._long_duration_ms
    
    @long_duration_ms.setter
    def long_duration_ms(self, value):
        self._long_duration_ms = value
    
    @property
    def is_toggle(self):
        return self._is_toggle
    
    @property
    def button(self):
        return self._button
        
        
class GamePad(EyeAngleProvider):
    def __init__(self, i2c, addr=0x50, pos_x_range=(0, 1023), pos_y_range=(0, 1023), \
                 x=(False, False), y=(False, False), a=(False, False), b=(False, False), \
                 select=(False, True), start=(False, True), debug=False):

        self._button_a = GameButton("A", default_value=a[0], is_toggle=a[1])
        self._button_b = GameButton("B", default_value=b[0], is_toggle=b[1])
        self._button_x = GameButton("X", default_value=x[0], is_toggle=x[1])
        self._button_y = GameButton("Y", default_value=y[0], is_toggle=y[1])
        self._button_select = GameButton("Select", default_value=select[0], is_toggle=select[1])
        self._button_start = GameButton("Start", default_value=start[0], is_toggle=start[1])
        
        self._x_range = pos_x_range
        self._y_range = pos_y_range

        self._pos_x = self._x_range[1] // 2 if self._x_range[1] > self._x_range[0] else self._x_range[0] // 2
        self._pos_y = self._y_range[1] // 2 if self._y_range[1] > self._y_range[0] else self._y_range[0] // 2
        
        # gamepad controller
        self._seesaw = Seesaw(i2c, addr)
        self._seesaw.pin_mode_bulk(BUTTON_MASK, self._seesaw.INPUT_PULLUP)

        # self.test_angle = 90
        # self.calibrating_delta = 1
        # self.calibrating = False
        # self.radar_enabled = False

        self._run = False
        self._debug = debug

    @property
    def pos_x(self):
        return self._pos_x
    
    @pos_x.setter
    def pos_x(self, value):
        self._pos_x = value
        
    @property
    def pos_y(self):
        return self._pos_y
    
    @pos_y.setter
    def pos_y(self, value):
        self._pos_y = value
        
    @property
    def button_a(self):
        return self._button_a
    
    @property
    def button_b(self):
        return self._button_b
    
    @property
    def button_x(self):
        return self._button_x
    
    @property
    def button_y(self):
        return self._button_y
    
    @property
    def button_select(self):
        return self._button_select
    
    @property
    def button_start(self):
        return self._button_start
    
    @property
    def debug(self):
        return self._debug
    
    @debug.setter
    def debug(self, value):
        self._debug = value

    @property
    def run(self):
        return self._run
    
    @run.setter
    def run(self, value):
        self._run = value
        
    def __str__(self):
        return f"GamePad(pos_x={self.pos_x}, pos_y={self.pos_y}, x={self._button_x.value}, y={self._button_y.value}, a={self._button_a.value}, b={self._button_b.value}, select={self._button_select.value}, start={self._button_start.value}, debug={self._debug})"
    
    def __repr__(self):
        return self.__str__()
    
    def xrange(self):
        return self._x_range
    
    def yrange(self):
        return self._y_range
        
    def _update(self, pos_x, pos_y, x, y, a, b, select, start, invert_x=False, invert_y=True):

        self._pos_x = 1023 - pos_x if invert_x else pos_x
        self._pos_y = 1023 - pos_y if invert_y else pos_y

        self._button_x.value = x
        self._button_y.value = y
        self._button_a.value = a
        self._button_b.value = b
        self._button_select.value = select
        self._button_start.value = start

    def read(self):
        if self._seesaw:
            raw_pos_x = self._seesaw.analog_read(JOY_X)
            raw_pos_y = self._seesaw.analog_read(JOY_Y)
            raw_buttons = self._seesaw.digital_read_bulk(BUTTON_MASK)
            self._update(
                raw_pos_x,
                raw_pos_y,
                not raw_buttons & (1 << BUTTON_X),
                not raw_buttons & (1 << BUTTON_Y),
                not raw_buttons & (1 << BUTTON_A),
                not raw_buttons & (1 << BUTTON_B),
                not raw_buttons & (1 << BUTTON_SELECT),
                not raw_buttons & (1 << BUTTON_START)
            )
    async def run(self, delay=0.01):
       if not self._run:
            self._run = True
            while self._run:
                try:
                    self.read()
                    if self._debug:
                        print(self)
                    await asyncio.sleep(delay)
                except Exception as e:
                    print(e)
                    self._run = False
        
    def stop(self):
        self._run = False
    
    async def get_angle_x(self, range_start: float, range_end: float):
        return map_range(self.pos_x, *self.xrange(), range_start, range_end)    
    
    async def get_angle_y(self, range_start: float, range_end: float):
        return map_range(self.pos_y, *self.yrange(), range_start, range_end)
