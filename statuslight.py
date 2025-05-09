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

    async def clear(self):
        self.pixel.fill((0, 0, 0))
        self.pixel.show()

    async def update(self, color: tuple[int, int, int]):
        self.pixel.fill(color)
        self.pixel.show()

    async def update_rgb(self, r: int, g: int, b: int):
        self.pixel.fill((r, g, b))
        self.pixel.show()

    async def update_index(self, index: int, color: tuple[int, int, int]):
        self.pixel[index] = color
        self.pixel.show()

    async def update_index_rgb(self, index: int, r: int, g: int, b: int):
        self.pixel[index] = (r, g, b)
        self.pixel.show()
