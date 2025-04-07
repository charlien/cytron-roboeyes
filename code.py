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


from adafruit_debouncer import Button


RED = (255, 0, 0)
YELLOW = (255, 150, 0)
GREEN = (0, 255, 0)
CYAN = (0, 255, 255)
BLUE = (0, 0, 255)
PURPLE = (180, 0, 255)
WHITE = (255, 255, 255)
OFF = (0, 0, 0)



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
min_pulse = 600
max_pulse = 2400

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
auto_blink = True
blink_range = (3, 7)

DEBUG_LOG = True


i2c = busio.I2C(board.GP17, board.GP16)
# radar_uart = busio.UART(board.GP0, board.GP1, baudrate=256000)
radar_uart = busio.UART(board.GP24, board.GP25, baudrate=256000)

button_pins: list[str] = ["GP20", "GP21"]
buttons: dict[str, Button] = {}

async def init_buttons():
    for pin in button_pins:
        button = digitalio.DigitalInOut(getattr(board, pin))
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

async def servo_test():
    global servos

    # Create a PWMOut object on the control pin.
    # pwm1 = pwmio.PWMOut(board.GP0, duty_cycle=0, frequency=50)
    # pwm2 = pwmio.PWMOut(board.GP1, duty_cycle=0, frequency=50)
    # pwm3 = pwmio.PWMOut(board.GP2, duty_cycle=0, frequency=50)
    # pwm4 = pwmio.PWMOut(board.GP3, duty_cycle=0, frequency=50)
    pwm5 = pwmio.PWMOut(board.GP4, duty_cycle=0, frequency=50)
    pwm6 = pwmio.PWMOut(board.GP5, duty_cycle=0, frequency=50)
    pwm7 = pwmio.PWMOut(board.GP6, duty_cycle=0, frequency=50)
    pwm8 = pwmio.PWMOut(board.GP7, duty_cycle=0, frequency=50)


    # # You might need to calibrate the min_pulse (pulse at 0 degrees) and max_pulse (pulse at 180 degrees) to get an accurate servo angle.
    # # The pulse range is 750 - 2250 by default (if not defined).

    # # Initialize Servo objects.
    # servo1 = servo.Servo(pwm1, min_pulse=min_pulse, max_pulse=max_pulse)
    # servo2 = servo.Servo(pwm2, min_pulse=min_pulse, max_pulse=max_pulse)
    # servo3 = servo.Servo(pwm3, min_pulse=min_pulse, max_pulse=max_pulse)
    # servo4 = servo.Servo(pwm4, min_pulse=min_pulse, max_pulse=max_pulse)
    servo5 = servo.Servo(pwm5, min_pulse=min_pulse, max_pulse=max_pulse)
    servo6 = servo.Servo(pwm6, min_pulse=min_pulse, max_pulse=max_pulse)
    servo7 = servo.Servo(pwm7, min_pulse=min_pulse, max_pulse=max_pulse)
    servo8 = servo.Servo(pwm8, min_pulse=min_pulse, max_pulse=max_pulse)

    # servos = [servo1, servo2, servo3, servo4, servo5, servo6, servo7, servo8]
    servos = [servo5, servo6, servo7, servo8]

    while True:
        # Sweep from 0 to 180
        for angle in range(0, 181, 10):
            for i, s in enumerate(servos):
                print(f'servo {i}: {angle}')
                s.angle = angle
            await asyncio.sleep(0.1)
        
        await asyncio.sleep(3)

        # Sweep from 180 back to 0
        for angle in range(180, -1, -10):
            for i, s in enumerate(servos):
                print(f'servo {i}: {angle}')
                s.angle = angle
            await asyncio.sleep(0.1)

        await asyncio.sleep(3)
        
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


async def main():

    import gc
    gc.collect()

    import supervisor # type: ignore
    supervisor.runtime.autoreload = False  # CirPy 8 and above
    print("supervisor.runtime.autoreload = False")

    await print_devices(i2c)

    radar_sensor = RadarSensor(radar_uart)


    tasks = []
    tasks.append(asyncio.create_task(gc_cleanup()))
    tasks.append(asyncio.create_task(servo_test()))
    tasks.append(asyncio.create_task(monitor_buttons()))
    tasks.append(asyncio.create_task(radar_sensor.run(debug=True)))

    await asyncio.gather(*tasks)

if __name__ == "__main__":
    asyncio.run(main()) # type: ignore