# cluttered map with lidar specks, similar to the user's
import numpy as np, os
from PIL import Image, ImageDraw
rng=np.random.default_rng(3); RES=0.05; W,H=int(4/RES),int(7/RES)
im=Image.new('L',(W,H),205); d=ImageDraw.Draw(im)
room=[(0.3,0.3),(3.6,0.4),(3.4,6.6),(0.2,6.5)]
px=lambda x,y:(x/RES,H-y/RES)
d.polygon([px(*p) for p in room],fill=254); d.line([px(*p) for p in room+room[:1]],fill=0,width=4)
for _ in range(12):
    x,y=rng.uniform(0.6,3.1),rng.uniform(0.8,6.0); s=rng.uniform(0.1,0.3)
    d.ellipse([px(x-s,y+s),px(x+s,y-s)],fill=0)
a=np.array(im)
free=np.argwhere(a==254)
for r,c in free[rng.choice(len(free),15,replace=False)]: a[r,c]=0   # single-pixel lidar specks
import pathlib; D=pathlib.Path(__file__).parent
Image.fromarray(a).save(D/'clutter.pgm')
open(D/'clutter.yaml','w').write("image: clutter.pgm\nmode: trinary\nresolution: 0.05\norigin: [0,0,0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n")
