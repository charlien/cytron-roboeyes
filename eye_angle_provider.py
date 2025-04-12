class EyeAngleProvider():
    async def get_angle_x(self, range_start: int, range_end: int):
        raise NotImplementedError("Subclasses must implement this method")
    
    async def get_angle_y(self, range_start: int, range_end: int):
        raise NotImplementedError("Subclasses must implement this method")
    
    
