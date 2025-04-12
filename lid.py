import servo # type: ignore

class Lid:
    def __init__(self, servo : servo.Servo, angle_close : int, angle_open: int):
        self.servo = servo
        self.angle_close = angle_close
        self.angle_open = angle_open
        
        self.last_angle = None
        self.is_opened = False
    
    def __str__(self):
        return f"Lid(servo={self.servo}, open={self.angle_open}, close={self.angle_close})"
    
    def __repr__(self):
        return self.__str__()
    
    def last(self):
        return f"{self.last_angle}"
    
    def open(self):
        self.move_direct(self.angle_open)
        self.is_opened = True
    
    def close(self):
        self.move_direct(self.angle_close)
        self.is_opened = False
    
    def move_direct(self, angle: int):
        if self._is_valid_angle(angle):
            self.servo.angle = angle
            self.last_angle = angle

    def _is_valid_angle(self, angle):
        return angle >= 0 and angle <= 180
        
    def opened(self):
        return self.is_opened

    def status(self):
        return f'angle:{self.last()} opened:{self.is_opened}'
    
    def toggle(self):
        if self.is_opened:
            self.close()
        else:
            self.open()