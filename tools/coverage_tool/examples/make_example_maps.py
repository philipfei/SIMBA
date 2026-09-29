"""Generate two small example ROS maps (.pgm + .yaml) for trying the tool."""
import math
import os

import numpy as np
from PIL import Image, ImageDraw

HERE = os.path.dirname(os.path.abspath(__file__))
RES = 0.05


def save(name, img, origin):
    Image.fromarray(img).save(os.path.join(HERE, name + '.pgm'))
    with open(os.path.join(HERE, name + '.yaml'), 'w') as f:
        f.write(f"image: {name}.pgm\nmode: trinary\nresolution: {RES}\n"
                f"origin: [{origin[0]}, {origin[1]}, 0.0]\nnegate: 0\n"
                "occupied_thresh: 0.65\nfree_thresh: 0.196\n")


def px(x, y, h):  # metres -> image pixel (row 0 at top)
    return x / RES, h - y / RES


def office():
    w, h = int(14 / RES), int(10 / RES)
    im = Image.new('L', (w, h), 205)  # unknown
    d = ImageDraw.Draw(im)
    room = [(1, 1), (13, 1), (13, 6), (9, 6), (9, 9), (1, 9)]  # L-shaped room
    d.polygon([px(x, y, h) for x, y in room], fill=254, outline=0)
    d.line([px(x, y, h) for x, y in room + room[:1]], fill=0, width=3)
    d.rectangle([px(3, 6.5, h), px(5.5, 5, h)], fill=0)       # table
    d.ellipse([px(9.6, 3.6, h), px(10.4, 2.8, h)], fill=0)    # pillar
    d.rectangle([px(1, 4.0, h), px(2.2, 3.4, h)], fill=0)     # cabinet at wall
    d.line([px(6.5, 1, h), px(6.5, 3.0, h)], fill=0, width=3)  # half wall
    save('office', np.asarray(im), (0.0, 0.0))


def rotated_hall():
    w, h = int(12 / RES), int(12 / RES)
    im = Image.new('L', (w, h), 205)
    d = ImageDraw.Draw(im)
    a = math.radians(17)
    cx, cy = 6, 6
    pts = []
    for x, y in [(-4.5, -3), (4.5, -3), (4.5, 3), (-4.5, 3)]:
        pts.append((cx + x * math.cos(a) - y * math.sin(a), cy + x * math.sin(a) + y * math.cos(a)))
    d.polygon([px(x, y, h) for x, y in pts], fill=254)
    d.line([px(x, y, h) for x, y in pts + pts[:1]], fill=0, width=3)
    d.ellipse([px(5.6, 6.4, h), px(6.4, 5.6, h)], fill=0)
    save('rotated_hall', np.asarray(im), (-2.0, -1.0))


if __name__ == '__main__':
    office()
    rotated_hall()
    print('wrote office.* and rotated_hall.*')
