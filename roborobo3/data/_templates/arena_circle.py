import numpy as np
from imageio import imwrite
import sys
from textwrap import dedent

radius = 200
margin = 5
arena_size = 2 * radius + 2 * margin + 1
center = arena_size // 2

arena = 255 * np.ones((arena_size, arena_size), dtype=np.uint8)
imwrite(f'env_arena_{radius*2}_circle_background.bmp', arena)

for i in range(arena.shape[0]):
    for j in range(arena.shape[1]):
        if (i - center) ** 2 + (j - center) ** 2 > radius ** 2:
            arena[i][j] = 0

imwrite(f'env_arena_{radius*2}_circle_environment.bmp', arena)


with open(f'arena_circle_{radius*2}.properties', 'w') as f:

    print(dedent(f"""
                gForegroundImageFilename = data/env_arena_{radius*2}_circle_environment.bmp
                gEnvironmentImageFilename = data/env_arena_{radius*2}_circle_environment.bmp
                gBackgroundImageFilename = data/env_arena_{radius*2}_circle_background.bmp
                gFootprintImageFilename = data/env_arena_{radius*2}_circle_environment.bmp
                gScreenWidth = {arena_size}
                gScreenHeight = {arena_size}


                # robot localisation

                # gAgentsInitArea* constrains localization to the designated area.
                # If not present, whole arena's area is considered
                # Ignored if agent localization is explicit
                gAgentsInitAreaX = {arena_size // 4}
                gAgentsInitAreaY = {arena_size // 4}
                gAgentsInitAreaWidth = {arena_size // 2}
                gAgentsInitAreaHeight = {arena_size // 2}


                # =-=-=-=-=-=

                # Physical objects

                gPhysicalObjectsVisible = true
                gPhysicalObjectsRedraw = false

                # gPhysicalObjectsInitArea* constrains localization to the designated area.
                # If not present, whole arena's area is considered (with a 10-pixel border)
                # Ignored if object localization is explicit
                gPhysicalObjectsInitAreaX = 10
                gPhysicalObjectsInitAreaY = 10
                gPhysicalObjectsInitAreaWidth = {arena_size - 10 - 10}
                gPhysicalObjectsInitAreaHeight = {arena_size - 10 - 10}
            """), file=f)
    nb = 20
    for i in range(nb):
        r = radius - 15
        x = int(np.round(center + r * np.cos(2 * np.pi * i / nb)))
        y = int(np.round(center + r * np.sin(2 * np.pi * i / nb)))
        print(f'physicalObject[{i}].x = {x}', file=f)
        print(f'physicalObject[{i}].y = {y}', file=f)

    for i in range(nb):
        r = radius - 15
        true_i = i + 1/2
        x = int(np.round(center + r * np.cos(2 * np.pi * true_i / nb)))
        y = int(np.round(center + r * np.sin(2 * np.pi * true_i / nb)))
        print(f'availableslot[{i}].x = {x}', file=f)
        print(f'availableslot[{i}].y = {y}', file=f)
