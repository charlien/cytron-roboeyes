import time
import board # type: ignore
import busio # type: ignore
import math
from micropython import const # type: ignore
import asyncio
import struct
from adafruit_debouncer import Debouncer, Button
from eye_angle_provider import EyeAngleProvider
from adafruit_simplemath import map_range

TARGET_COUNT = const(3)
# HLK-LD2450 Protocol Constants
FRAME_HEADER = bytearray([0xAA,0xFF, 0x03, 0x00])
FRAME_HEADER_LENGTH = const(4)

BODY_LENTH = const(TARGET_COUNT * 8)
FRAME_TAIL = bytearray([0x55, 0xCC])
TAIL_LENGTH = const(2)
SIGN_VALUE = const(1 << 15)
MAX_VALUE = const((1 << 15) - 1)
MIN_VALUE = const(-MAX_VALUE)
BUFFER_SIZE = const(FRAME_HEADER_LENGTH + BODY_LENTH + TAIL_LENGTH)

class Targets:
    def __init__(self, angle_range=(150,30)):
        self._targets = []
        self._angle_range = angle_range
    
    @property
    def targets(self):
        return self._targets
    
    @targets.setter
    def targets(self, value):
        self._targets = value

    @property
    def closest(self):
        if len(self._targets) == 0: 
            return None        
        return min(self._targets,  key=lambda t: t['y'])
    
    @property
    def angle_range(self):
        return self._angle_range
    
    def __str__(self):
        return f"Targets(targets={self.targets}, closest={self.closest})"
    
    def __repr__(self):
        return self.__str__()
    
class KalmanFilter:
    def __init__(self, process_variance=0.1, measurement_variance=0.1):
        """
        Initialize a 1D Kalman filter.
        
        :param process_variance: Variance in the process (how much we expect the value to change)
        :param measurement_variance: Variance in the measurement (how noisy our measurements are)
        """
        self.process_variance = process_variance
        self.measurement_variance = measurement_variance
        
        # Initialize state
        self.estimate = 0.0
        self.estimate_error = 1.0
        
    def update(self, measurement):
        """
        Update the filter with a new measurement.
        
        :param measurement: New measurement value
        :return: Filtered value
        """
        # Prediction step
        prediction_error = self.estimate_error + self.process_variance
        
        # Update step
        kalman_gain = prediction_error / (prediction_error + self.measurement_variance)
        self.estimate = self.estimate + kalman_gain * (measurement - self.estimate)
        self.estimate_error = (1 - kalman_gain) * prediction_error
        
        return self.estimate
    
class RadarSensor(EyeAngleProvider):
    def __init__(self, uart: busio.UART, max_errors=20):
        """
        Initialize the HLK-LD2450 radar sensor.
        
        :param uart: The UART bus object to communicate with the sensor
        """
        self._uart = uart
        self._targets_handler = Targets()

        # Initialize Kalman filters for angles
        self.x_filter = KalmanFilter(process_variance=0.1, measurement_variance=0.5)
        self.y_filter = KalmanFilter(process_variance=0.1, measurement_variance=0.5)
        self.angle_filter = KalmanFilter(process_variance=0.1, measurement_variance=0.5)

        self._is_running = False
        self.buffer_size = BUFFER_SIZE
        self.buffer = bytearray(self.buffer_size)
        self.target_ready = asyncio.Event()

        self.x_min = MAX_VALUE
        self.x_max = MIN_VALUE

        self.y_min = MAX_VALUE
        self.y_max = MIN_VALUE

        self.angle_min = 180
        self.angle_max = 0

        self._max_errors = max_errors

        self.MAX_SAMPLES = 1 << 16

        self._samples = 0

        self._errors = 0

        self._debug_trigger = None
        self._debug_time = 0
    def __str__(self):
        return f"RadarSensor(targets={self._targets_handler} samples={self._samples} errors={self._errors})"
    
    def __repr__(self):
        return self.__str__()
    
    @property
    def failed(self):
        return self._errors >= self._max_errors
    
    @property
    def running(self):
        return self._is_running
    
    @property
    def closest(self):
        return self._targets_handler.closest
    
    @property
    def targets(self):
        return self._targets_handler.targets
    
    @property
    def targets_angle_range(self):
        return self._targets_handler.angle_range
    
    @property
    def samples(self):
        return self._samples
    
    @property
    def errors(self):
        return self._errors
    
    @property
    def angle_range(self):
        return self._targets_handler.angle_range
    
    
    def update_filters(self, x, y, angle):
        return self.x_filter.update(x), self.y_filter.update(y), self.angle_filter.update(angle)
    
    async def stop(self):
        if not self._is_running: return
        self._is_running = False
        self._targets_handler.targets = []
        print("Radar stopped called")

    def _trigger_fn(self, interval_seconds=3):
        current_time = time.time()
        result = current_time - self._debug_time > interval_seconds
        if current_time - self._debug_time > interval_seconds + 1: self._debug_time = current_time
        # print(f"current_time: {current_time}, self._debug_time: {self._debug_time} result: {result}")
        return result

    async def run(self, debug=False, sampling_rate=0.01):
        if self._is_running: return
        self._is_running = True
        if debug and self._debug_trigger is None:
            self._debug_time = time.time()
            # The issue is here - lambda is passing the function reference, not calling it
            # Change from lambda: self._trigger_fn to lambda: self._trigger_fn()
            self._debug_trigger = Debouncer(lambda: self._trigger_fn(interval_seconds=5), interval=sampling_rate)
        print(f"Radar sensor started, debug: {True if self._debug_trigger else False}")
        while self._is_running:
            try:
                await self.read_uart()
            except RuntimeError as e:
                print(e)
                break

            await asyncio.sleep(sampling_rate)
            
        print("Radar sensor stopped")
            
    async def read_uart(self):
        if not self._is_running:
            self._uart.reset_input_buffer()
            return
        
        if self._debug_trigger:
            self._debug_trigger.update()
            # print(f"value: {self._debug_trigger.value} fell: {self._debug_trigger.fell} rose: {self._debug_trigger.rose}")
            if self._debug_trigger.fell or self._debug_trigger.rose:
                print(f"uart: {self._uart.in_waiting}")

        """Process the current frame of radar data."""
        # Process as many frames as available to improve speed

        while self._uart.in_waiting >= self.buffer_size and self._is_running:
            self._uart.readinto(self.buffer)
            if self.buffer[:4] == FRAME_HEADER and self.buffer[-2:] == FRAME_TAIL:
                frame_data = self.buffer[4:-2]
                # Create a task to parse the frame asynchronously
                asyncio.create_task(self._process_frame(frame_data))
                self._errors = 0
            else:
                self._errors += 1
                # Clear any remaining data to try to resynchronize
                self._resynchronize()
                # Yield to other tasks to prevent blocking
            await asyncio.sleep(0)
            
    
    async def _process_frame(self, frame_data):
        """Process a single frame of data asynchronously."""
        targets = await self.parse_frame(frame_data)
        if self._debug_trigger: print(f"targets: {targets}")
        self._targets_handler.targets = targets

    
    def _resynchronize(self):
        """Attempt to resynchronize with the radar data stream."""
        print("Resynchronizing")
        # Create a new buffer to hold potential frame data
        temp_buffer = bytearray(self.buffer_size)
        
        # Read bytes one at a time until we find a valid header
        bytes_read = 0
        while self._uart.in_waiting > 0 and bytes_read < self.buffer_size:
            # Read one byte
            byte = bytearray(1)
            self._uart.readinto(byte)
            
            # Add the byte to our temporary buffer
            temp_buffer[bytes_read] = byte[0]
            bytes_read += 1
            
            # If we have at least 4 bytes, check if we have a valid header
            if bytes_read >= 4 and temp_buffer[bytes_read-4:bytes_read] == FRAME_HEADER:
                # We found a header! Now read the rest of the frame
                remaining = self.buffer_size - bytes_read
                if self._uart.in_waiting >= remaining:
                    # Read the rest of the frame
                    self._uart.readinto(temp_buffer[bytes_read:])
                    # Copy the valid frame to our main buffer
                    self.buffer = temp_buffer
                break
    
    def update_min_max(self, x, y, angle):
        self.x_min, self.x_max = self.min_max(x, self.x_min, self.x_max)
        self.y_min, self.y_max = self.min_max(y, self.y_min, self.y_max)
        self.angle_min, self.angle_max = self.min_max(angle, self.angle_min, self.angle_max)
    
    def unpack_target_data(self, raw_target_data):
        x, y, speed, distance_resolution = struct.unpack('<HHHH', raw_target_data)
        x = -x if x < SIGN_VALUE else x - SIGN_VALUE
        y = -y if y < SIGN_VALUE else y - SIGN_VALUE
        speed = -speed if speed < SIGN_VALUE else speed - SIGN_VALUE
        return x, y, speed, distance_resolution

    def min_max(self, value, min_value, max_value):
        return min(value, min_value), max(value, max_value)
    
    def compute_angle_degrees(self, x, y):
        return math.degrees(math.atan2(y, x))
    
    async def parse_frame(self, frame):
        targets = []
        self._samples = self._samples + 1 if self._samples < self.MAX_SAMPLES else 0
        for i in range(TARGET_COUNT):
            # Each target takes 8 bytes
            target_data = frame[i*8:(i+1)*8]
            
            x, y, speed, distance_resolution = self.unpack_target_data(target_data)
                        
            # Only add target if it's valid (non-zero coordinates)
            if x != 0 or y != 0 or distance_resolution != 0:
                # Update kalman filters
                angle = self.compute_angle_degrees(x, y)

                # self.update_min_max(x, y, angle)
                # x_Filter, y_Filter = self.update_filters(x, y)

                # Add target to list
                targets.append({
                    'id': i,
                    'x': x,
                    'y': y,
                    'angle': angle,
                    # 'x': (x, x_Filter, self.x_min, self.x_max),
                    # 'y': (y, y_Filter, self.y_min, self.y_max),
                    # 'x': (x, (self.x_min, self.x_max)),
                    # 'y': (y, (self.y_min, self.y_max)),
                    # 'angle': (angle, (self.angle_min, self.angle_max)),
                    'speed': speed,
                })

        return targets
    
    async def get_angle_x(self, range_start: float, range_end: float):
        return map_range(self.closest['angle'], *self.angle_range, range_start, range_end)
    
    async def get_angle_y(self, range_start: float, range_end: float):
        raise NotImplementedError("Radar doesn't provide an angle for the y axis")