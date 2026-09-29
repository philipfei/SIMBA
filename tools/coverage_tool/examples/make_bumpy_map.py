# user-like map: slightly rotated room, bumpy lidar walls, irregular clutter clusters
import numpy as np, os, pathlib
D=pathlib.Path(__file__).parent
from PIL import Image, ImageDraw
rng=np.random.default_rng(7); RES=0.05; X0,Y0=-1.2,-4.8; W,H=int(4.0/RES),int(6.9/RES)
im=Image.new('L',(W,H),205); d=ImageDraw.Draw(im)
px=lambda x,y:((x-X0)/RES,H-(y-Y0)/RES)
room=[(-1.0,1.55),(2.35,1.75),(2.5,-4.6),(-0.75,-4.75)]
d.polygon([px(*p) for p in room],fill=254)
# bumpy walls: many small blobs along each wall
for (a,b) in zip(room,room[1:]+room[:1]):
    a,b=np.array(a),np.array(b)
    for t in np.linspace(0,1,120):
        p=a+t*(b-a); s=rng.uniform(0.03,0.09) if rng.random()<0.35 else 0.04
        d.ellipse([px(p[0]-s,p[1]+s),px(p[0]+s,p[1]-s)],fill=0)
# clutter clusters (chairs/table legs): groups of small irregular blobs
for _ in range(9):
    cx,cy=rng.uniform(-0.3,2.0),rng.uniform(-4.2,1.1)
    for _ in range(rng.integers(3,9)):
        x,y=cx+rng.normal(0,0.12),cy+rng.normal(0,0.12); s=rng.uniform(0.03,0.08)
        d.rectangle([px(x-s,y+s),px(x+s,y-s)],fill=0)
for x,y in [(-0.3,0.75),(1.2,-0.35),(0.75,-3.3)]:
    d.rectangle([px(x-0.05,y+0.05),px(x+0.05,y-0.05)],fill=0)
a=np.array(im)
Image.fromarray(a).save(D/'bumpy.pgm')
open(D/'bumpy.yaml','w').write(f"image: bumpy.pgm\nmode: trinary\nresolution: 0.05\norigin: [{X0},{Y0},0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n")
