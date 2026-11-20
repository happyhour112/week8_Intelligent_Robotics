"""Tiny GRID-ONLY classroom world. Coordinates are cells, not metres.
This is intentionally NOT a Gazebo physics world nor SLAM.
"""
from math import hypot

W, H = 19, 13
RES = 0.25  # metre per cell; coarse and illustrative only
START = (1, 2)
TOYS = ((8, 2), (15, 6))
BOX = (17, 10)
WALLS = {
    (4,5), (5,2), (5,3), (5,4), (5,5), (5,6), (5,7), (5,8), (5,9), (5,10), (6,5),
    (9,1), (9,3), (9,4), (9,5), (9,6), (9,7), (9,8), (9,10), (9,11), (10,7), (11,3),
    (13,1), (13,2), (13,3), (13,5), (13,6), (13,7), (13,8), (13,10), (13,11),
    (16,1), (16,2), (16,3), (16,4), (16,5), (16,8), (16,9), (16,10), (16,11),
}
PERIMETER = ({(x, 0) for x in range(W)} | {(x, H-1) for x in range(W)} |
             {(0, y) for y in range(H)} | {(W-1, y) for y in range(H)})
WALLS |= PERIMETER


def inside(c):
    return 0 <= c[0] < W and 0 <= c[1] < H


def center(cell):
    """Grid (x,y) -> world (metres) at the CELL CENTER."""
    return ((cell[0] + 0.5) * RES, (cell[1] + 0.5) * RES)


def neighbors(c):
    x,y = c
    # 4-connected: simpler than Nav2 SmacPlanner2D's 8-connected model
    return ((x+1,y), (x,y+1), (x-1,y), (x,y-1))


def visible(src, target):
    """Conservative grid line-of-sight using small ray-march steps.
    Do not use this as a real sensor model: no noise, reflection or occlusion physics.
    """
    x0,y0 = (src[0]+.5, src[1]+.5)
    x1,y1 = (target[0]+.5, target[1]+.5)
    n = max(1, int(hypot(x1-x0,y1-y0)*25))
    for k in range(1,n+1):
        t = k/n
        p=(int(x0+(x1-x0)*t), int(y0+(y1-y0)*t))
        if p in WALLS and p != target:
            return False
    return True


def reveal(known, robot, range_cells=3.25):
    """Simulate a finite-view sensor, modifying known (cell -> 0 or 100).
    -1 means NOT observed. This is NOT SLAM/localization.
    """
    changed=0
    for y in range(H):
        for x in range(W):
            c=(x,y)
            if hypot(x-robot[0], y-robot[1]) > range_cells:
                continue
            if visible(robot,c):
                value=100 if c in WALLS else 0
                if known.get(c) != value:
                    known[c]=value
                    changed+=1
    return changed
