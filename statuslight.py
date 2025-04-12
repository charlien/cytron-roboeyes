import neopixel # type: ignore


COLORS = {}
COLORS['red'] = (255, 0, 0)
COLORS['yellow'] = (255, 150, 0)
COLORS['green'] = (0, 255, 0)
COLORS['cyan'] = (0, 255, 255)
COLORS['blue'] = (0, 0, 255)
COLORS['purple'] = (180, 0, 255)


class StatusLight:
    def __init__(self, pixel: neopixel):
        self.pixel = pixel

    async def change_color(self, color: tuple[int, int, int]):
        self.pixel.fill(color)
        self.pixel.show()
