import hashlib
import numpy as np
import pytest
import yaml
from PIL import Image
from simba_coverage.geometry import Grid
from simba_coverage.plan_file import load_plan, map_image_sha256


@pytest.fixture
def room_map(tmp_path):
    a=np.full((80,80),254,np.uint8);a[[0,-1],:]=0;a[:,[0,-1]]=0
    Image.fromarray(a).save(tmp_path/'map.pgm')
    (tmp_path/'map.yaml').write_text('image: map.pgm\nresolution: 0.05\norigin: [0, 0, 0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')
    return tmp_path/'map.yaml'


def write_plan(tmp_path,points,**extra):
    poses=[{'x':x,'y':y,'yaw':0.,'orientation':{'z':0.,'w':1.},'kind':'lane'} for x,y in points]
    path=tmp_path/'plan.yaml';path.write_text(yaml.safe_dump({'frame_id':'map','poses':poses,**extra}))
    return path


def test_safe_plan_is_split_into_ordered_targets(tmp_path,room_map):
    grid=Grid.load(room_map)
    path=write_plan(tmp_path,[[1.,1.],[1.,1.],[3.,1.],[3.,3.]],map_image_sha256=map_image_sha256(room_map))
    plan=load_plan(path,grid,room_map,'map',.2,1.)
    assert plan['points']==[[1.,1.],[3.,1.],[3.,3.]]
    assert plan['targets'][0].start==[1.,1.] and plan['targets'][-1].end==[3.,3.]
    assert all(t.kind=='plan' for t in plan['targets']) and len(plan['targets'])==4
    assert plan['plan_sha256']==hashlib.sha256(path.read_bytes()).hexdigest()


def test_plan_hugging_a_wall_is_rejected(tmp_path,room_map):
    grid=Grid.load(room_map)
    with pytest.raises(ValueError,match='PLAN_TOO_CLOSE_TO_OBSTACLES: 1 of 2'):
        load_plan(write_plan(tmp_path,[[1.,1.],[2.,1.],[2.,.2]]),grid,room_map,'map',.2,1.)


def test_plan_from_another_map_or_frame_is_rejected(tmp_path,room_map):
    grid=Grid.load(room_map)
    with pytest.raises(ValueError,match='PLAN_MAP_MISMATCH'):
        load_plan(write_plan(tmp_path,[[1.,1.],[2.,1.]],map_image_sha256='0'*64),grid,room_map,'map',.2,1.)
    with pytest.raises(ValueError,match='PLAN_FRAME_MISMATCH'):
        load_plan(write_plan(tmp_path,[[1.,1.],[2.,1.]],frame_id='odom'),grid,room_map,'map',.2,1.)
    with pytest.raises(ValueError,match='fewer than two'):
        load_plan(write_plan(tmp_path,[[1.,1.]]),grid,room_map,'map',.2,1.)
