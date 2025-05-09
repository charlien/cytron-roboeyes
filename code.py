"""
DESCRIPTION:
This example code will uses MOTION 2350 Pro
to control the Servos.

AUTHOR  : Cytron Technologies Sdn Bhd
WEBSITE  : www.cytron.io
EMAIL    : support@cytron.io

REFERENCE:
Tutorial link:
https://cytron.io/tutorial/get-started-motion-2350-pro-circuitpython-micro-servo-motor
"""
# Import neccessary libraries
import time
import board # type: ignore
import digitalio # type: ignore
import pwmio # type: ignore
from radar import RadarSensor
import servo # type: ignore
import asyncio # type: ignore
import busio # type: ignore
import microcontroller # type: ignore
import neopixel # type: ignore
import random

from adafruit_debouncer import Button

from WiFiManager import WiFiManager
from eyeball import EyeBall
from gamepad import GamePad
from lid import Lid
from statuslight import StatusLight, COLORS

status_light = StatusLight(neopixel.NeoPixel(board.GP23, 2))


"""
eyeball mapping (back view)

       100
        |
  134 ------- 40 (look right)
        |
        35 
"""

EYEBALL_HORIZONTAL_RANGE = (40, 135)
EYEBALL_VERTICAL_RANGE = (35, 100)


# need to give some tolerance since not all PCA boards are the same or else
# the servos MG90s can actually go from 500 too 2500ms
# [600-2400] translates to 599.6 us to 2.398 ms with the pi pico 2 
MIN_PULSE = 600
MAX_PULSE = 2400

# servos
# 0 - eye x
# 1 - eye y
# 2 - lid top left (front view)
# 3 - lid bottom left (front view)
# 4 - lid top right (front view)
# 5 - lid bottom right  (front view)

servos = []
lids = []
last_blink = 0
auto_blink = False
blink_range = (3, 7)

DEBUG_LOG = True


i2c = busio.I2C(board.GP17, board.GP16)
# radar_uart = busio.UART(board.GP0, board.GP1, baudrate=256000)
radar_uart = busio.UART(board.GP24, board.GP25, baudrate=256000)

button_pins: list[int] = [20, 21]
buttons: dict[int, Button] = {}



def get_gpio(pin: int):
    return getattr(board, f"GP{pin}")

def get_pwm(pin: int, duty_cycle: int = 0, frequency: int = 50):
    return pwmio.PWMOut(get_gpio(pin), duty_cycle = duty_cycle, frequency = frequency)

def configure_servo(pin: int, min_pulse_us, max_pulse_us):
    return servo.Servo(get_pwm(pin), min_pulse=min_pulse_us, max_pulse=max_pulse_us)

async def init_servos(ports: list[int] = [0, 1, 2, 3, 4, 5, 6, 7], min_pulse=MIN_PULSE, max_pulse=MAX_PULSE):
    for port_num in ports:
        servos.append(configure_servo(port_num, min_pulse, max_pulse))

async def init_buttons():
    for pin in button_pins:
        button = digitalio.DigitalInOut(get_gpio(pin))
        button.direction = digitalio.Direction.INPUT
        button.pull = digitalio.Pull.UP
        buttons[pin] = Button(button)

async def monitor_buttons(period_sec=0.001):
    # lazy init if buttons are not initialized
    if buttons == {}:
        await init_buttons()

    while True:
        for pin in button_pins:
            buttons[pin].update()
            if buttons[pin].pressed:
                print(f"Button {pin} pressed")
        await asyncio.sleep(period_sec)

async def toggle_lids(lids: list['Lid']) -> None:
    """
    Toggle all of the provided lids.

    :param lids: A list of Lid objects to toggle
    :return: None
    """
    for lid in lids:
        lid.close()

    await asyncio.sleep(0.3)
    
    for lid in lids:
        lid.open()

    await asyncio.sleep(0.3)

async def handle_lids(pad: GamePad, radar: RadarSensor, lids: list['Lid']):
    global auto_blink, last_blink, blink_range

    now = time.monotonic()
    if pad.button_x.value:
        await toggle_lids(lids)
    elif pad.button_y.value:
        await toggle_lids(lids[:2])
    elif pad.button_a.value:
        await toggle_lids(lids[2:])
    elif pad.button_b.value:
        auto_blink = not auto_blink
    else:
        if auto_blink and now - last_blink > random.uniform(*blink_range):
            if radar.running and radar.closest is None:
                await close_lids(lids)
            else:
                last_blink = now
                await toggle_lids(lids)

async def calibrate(pad: GamePad, servo):
    await status_light.update(COLORS['blue'])
    if servo:
        if pad.button_x.value:
            # pad.reset_test_angle()
            await open_lids(lids)
            servo.angle = 180
        elif pad.button_y.value:
            # pad.decrement_test_angle()
            pass
        elif pad.button_a.value:
            # pad.increment_test_angle()
            pass
        if pad.button_b.value:
            # pad.toggle_calibrating_delta()
            await close_lids(lids)
            servo.angle = 0


async def populate_lids(settings = None):
    # Setup lids
    lid_top_left = Lid(servos[2], angle_close=100, angle_open=180)
    lids.append(lid_top_left)

    lid_bottom_left = Lid(servos[3], angle_close=80, angle_open=0)
    lids.append(lid_bottom_left)

    lid_top_right = Lid(servos[4], angle_close=180, angle_open=100)
    lids.append(lid_top_right)

    lid_bottom_right = Lid(servos[5], angle_close=0, angle_open=80)
    lids.append(lid_bottom_right)

async def close_lids(lids: list['Lid']):
    if lids:
        if DEBUG_LOG: print("closing lids")
        for lid in lids:
            lid.close()
        await asyncio.sleep(0.3)

async def open_lids(lids: list['Lid']):
    if lids:
        if DEBUG_LOG: print("opening lids")
        for lid in lids:
            lid.open()
        await asyncio.sleep(0.3)


async def move_eyeball(pad: GamePad, eyeball: EyeBall, radar: RadarSensor):

    while True:
        if pad.button_select.value:
            await calibrate(pad, servos[7])
        else:
            lid_task = asyncio.create_task(handle_lids(pad, radar, lids))
            eye_task = asyncio.create_task(eyeball.update(pad, radar, status_light))

            await asyncio.gather(lid_task, eye_task)
            
            if DEBUG_LOG and eyeball.has_changed():
                print(eyeball.status())

        await asyncio.sleep(0.01)

async def print_devices(i2c_bus):
    while not i2c_bus.try_lock():
        print("waiting for i2c...")
        await asyncio.sleep(0.1)
    try:
        print(
            "I2C addresses found:",
            [hex(device_address) for device_address in i2c_bus.scan()],
        )
    finally:  # unlock the i2c bus when ctrl-c'ing out of the loop
        i2c_bus.unlock()


async def gc_cleanup(interval=10):
    import gc # type: ignore
    while True:
        gc.collect()
        if DEBUG_LOG: print(f'free: {gc.mem_free() / 1000:.2f}kB allocated: {gc.mem_alloc()/ 1000:.2f}kB')        
        await asyncio.sleep(interval)

async def start_wifi(tx_pin, rx_pin, port=8080):
    wifi_manager = WiFiManager(get_gpio(tx_pin), get_gpio(rx_pin), port=port) # uart 1
    await wifi_manager.start_ap("RoboEyes")
    await wifi_manager.process_uart_data()

async def main():

    import gc
    gc.collect()

    tasks = []

    import supervisor # type: ignore
    supervisor.runtime.autoreload = False  # CirPy 8 and above
    print("supervisor.runtime.autoreload = False")
    tasks.append(asyncio.create_task(gc_cleanup()))


    await status_light.update(COLORS['red'])
    await print_devices(i2c)
    await init_servos()
    await populate_lids()

    eye = EyeBall(servos[0], servos[1], *EYEBALL_HORIZONTAL_RANGE, *EYEBALL_VERTICAL_RANGE, debug=True)
    gamepad = GamePad(i2c, debug=False)
    radar_sensor = RadarSensor(radar_uart)

    # tasks.append(asyncio.create_task(start_wifi(28, 29)))
    tasks.append(asyncio.create_task(monitor_buttons()))
    tasks.append(asyncio.create_task(gamepad.run()))
    tasks.append(asyncio.create_task(move_eyeball(gamepad, eye, radar_sensor)))

    
    await asyncio.gather(*tasks)

if __name__ == "__main__":
    asyncio.run(main()) # type: ignore