import numpy as np
from imageio import imwrite
import sys

arena = 255 * np.ones((600, 600), dtype=np.uint8)
imwrite('env_arena_600_circle_background.bmp', arena)

for i in range(arena.shape[0]):
    for j in range(arena.shape[1]):
        if (i - 300) ** 2 + (j - 300) ** 2 > 280 ** 2:
            arena[i][j] = 0

imwrite('env_arena_600_circle_environment.bmp', arena)
