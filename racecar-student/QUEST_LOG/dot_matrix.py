LETTERS = {
    "a": (12157, 3),
    "b": (27630, 3),
    "c": (14627, 3),
    "d": (27502, 3),
    "e": (31207, 3),
    "f": (31204, 3),
    "g": (14635, 3),
    "h": (23533, 3),
    "i": (29847, 3),
    "j": (29838, 3),
    "k": (23469, 3),
    "l": (18727, 3),
    "m": (18732721, 5),
    "n": (646073, 4),
    "o": (11114, 3),
    "p": (27620, 3),
    "q": (432549, 4),
    "r": (27637, 3),
    "s": (14798, 3),
    "t": (29842, 3),
    "u": (23402, 3),
    "v": (23418, 3),
    "w": (18535754, 5),
    "x": (23213, 3),
    "y": (23186, 3),
    "z": (29351, 3),
    "-": (448, 3),
    ".": (2, 3),
    "0": (31599, 3),
    "1": (25751, 3),
    "2": (10919, 3),
    "3": (25550, 3),
    "4": (23497, 3),
    "5": (31118, 3),
    "6": (14762, 3),
    "7": (29348, 3),
    "8": (11242, 3),
    "9": (15305, 3)
}


def to_array(bitstring, width):
    nbits = 5 * width
    bits = bin(bitstring)[2:].zfill(nbits)

    return [
        [int(bit) for bit in bits[r * width:(r + 1) * width]]
        for r in range(5)
    ]


class DotMatrixTextDisplay:
    DISPLAY_WIDTH = 24
    DISPLAY_HEIGHT = 8

    def __init__(self, text, fps=5):
        self.text = text.lower()
        self.fps = fps
        self.frame = 0

        # one long 5 row bitmap
        self.bitmap = [[] for _ in range(5)]

        #space before text scrolls on.
        for row in self.bitmap:
            row.extend([0] * self.DISPLAY_WIDTH)

        for ch in self.text:
            if ch == " ":
                letter = [[0] * 3 for _ in range(5)]
                width = 3
            elif ch in LETTERS:
                bits, width = LETTERS[ch]
                letter = to_array(bits, width)
            else:
                continue

            for r in range(5):
                self.bitmap[r].extend(letter[r])
                self.bitmap[r].append(0)  # 1 col spacing

        #blank space after text scrolls off.
        for row in self.bitmap:
            row.extend([0] * self.DISPLAY_WIDTH)

        self.total_frames = len(self.bitmap[0]) - self.DISPLAY_WIDTH + 1

    def get_current_frame(self):
        frame = [[0] * self.DISPLAY_WIDTH for _ in range(self.DISPLAY_HEIGHT)]

        x0 = int(self.frame)

        y_offset = 1 # Vertically center the 5-pixel font in the 8-pixel display.

        for r in range(5):
            frame[y_offset + r] = self.bitmap[r][x0:x0 + self.DISPLAY_WIDTH]

        return frame

    def increment_frame(self, delta_t):
        self.frame = (self.frame + delta_t * self.fps) % self.total_frames