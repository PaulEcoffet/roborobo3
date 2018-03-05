import numpy as np
from imageio import imwrite
import sys
import argparse

if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("x", type=int)
    ap.add_argument("y", type=int)
    ap.add_argument("--nrow", type=int, default=1)
    ap.add_argument("--ncol", type=int, default=1)
    ap.add_argument("--wallwidth", type=int, default=4)
    ap.add_argument("--dest", type=str, default=".")

    a = ap.parse_args()
    width = a.wallwidth + (a.x + a.wallwidth) * a.ncol
    height =  a.wallwidth + (a.y + a.wallwidth) * a.nrow
    arena = 255 * np.ones((height, width), dtype=np.uint8)
    imwrite('env_arena_background_{}_{}_{}_{}.bmp'.format(a.ncol, a.nrow, a.x, a.y), arena)
    for i in range(a.ncol):
        start = (a.x + a.wallwidth) * i
        arena[:, start:(start+a.wallwidth)] = 0
    arena[:, -a.wallwidth:] = 0 # last vertical wall
    for i in range(a.nrow):
        start = (a.y + a.wallwidth) * i
        arena[start:(start+a.wallwidth), :] = 0
    arena[-a.wallwidth:, :] = 0 # last vertical wall
    imwrite('env_arena_environment_{}_{}_{}_{}.bmp'.format(a.ncol, a.nrow, a.x, a.y), arena)
