"""Behavioral regression tests for shared configuration and revised coverage planning."""
import copy
import shutil
import importlib.util
import itertools
import math
from pathlib import Path
import numpy as np
import pytest
import yaml
from simba_coverage.geometry import Grid
from simba_coverage.params import load_settings,canonical_hash,offline_case,resource
from simba_coverage.planning import (rank_candidates,merge_intervals,stripes,wall_evidence,
                                     optimize,boundary_routes,stroke_union,trim_stripes,points_of)


def room():
    cells=np.zeros((32,40),np.int8);cells[[0,-1],:]=100;cells[:,[0,-1]]=100
    return Grid.from_cells(cells,.1,(.3,-.4,.17))


def candidate(angle,offset,n,fragments=3,length=8.,seconds=60.):
    return dict(angle_deg=angle,offset_m=offset,coverage_cells=n,fragment_count=fragments,
                fragment_length_m=length,estimated_time_s=seconds,valid=True)


def test_one_global_tolerance_band_order_independent():
    original=[candidate(0,0,10000,5),candidate(5,.1,9995,2),candidate(90,.2,9989,1)]
    for perm in itertools.permutations(original):
        data=copy.deepcopy(perm);winner=rank_candidates(data,.001)
        assert winner['angle_deg']==5 and winner['selection_basis']=='within_tolerance'
        assert all(c['global_max_cells']==10000 and c['tolerance_cells']==10 for c in data)
        assert next(c for c in data if c['coverage_cells']==9989)['eligible'] is False


def test_global_band_does_not_chain_local_tolerances():
    data=[candidate(0,0,10000,10),candidate(0,.1,9990,5),candidate(90,0,9981,1)]
    assert rank_candidates(data,.001)['coverage_cells']==9990
    assert rank_candidates(copy.deepcopy(data),0)['coverage_cells']==10000
    tiny=[candidate(0,0,99,9),candidate(90,0,98,1)]
    assert rank_candidates(tiny,.001)['coverage_cells']==99


def test_ranking_final_tie_uses_offset_before_direction():
    data=[candidate(0,.1,100),candidate(90,0,100),candidate(0,0,100)]
    assert rank_candidates(data,.001)['angle_deg']==0
    assert rank_candidates(data,.001)['offset_m']==0


@pytest.mark.parametrize('intervals,expected,dropped',[
    ([(0,.1),(.2,.3),(.4,.5)],[[0,.5]],[]),
    ([(0,.1),(.2,.3)],[],[[0,.3]]),
    ([(0,.1),(.2,.35)],[[0,.35]],[]),
    ([(0,.1),(.2,.8)],[[0,.8]],[]),
    ([(0,.4),(.5,.6),(.7,1.2)],[[0,.4],[.5,1.2]],[]),
    ([(0,.4),(.45,.5),(.9,1.4)],[[0,.5],[.9,1.4]],[]),
])
def test_deterministic_trim_merge(intervals,expected,dropped):
    for perm in itertools.permutations(intervals):
        kept,left=merge_intervals(perm,.35)
        assert np.allclose(kept,expected) and np.allclose(left,dropped)


def test_config_hash_ignores_comments_order_and_numeric_spelling(tmp_path):
    shutil.copytree(resource('.'),tmp_path/'config',dirs_exist_ok=True)
    base=load_settings();path=tmp_path/'config'/'coverage_params.yaml'
    raw=yaml.safe_load(path.read_text());params=raw['/coverage_config']['ros__parameters']
    raw=dict(reversed(list(raw.items())))
    path.write_text('# Commentary only\n'+yaml.safe_dump(raw,sort_keys=False))
    assert load_settings(tmp_path/'config').hash==base.hash
    robot_path=tmp_path/'config'/'robot_params.yaml';robot=yaml.safe_load(robot_path.read_text())
    robot['/simba_defaults']['ros__parameters']['motion']['linear_m_s']=.11
    robot_path.write_text(yaml.safe_dump(robot))
    changed=load_settings(tmp_path/'config')
    assert changed.hash!=base.hash
    rendered=yaml.safe_load(changed.render_nav2('/tmp/map.yaml',tmp_path/'rendered').read_text())
    assert rendered['controller_server']['ros__parameters']['FollowPath']['desired_linear_vel']==.11
    navigator=rendered['bt_navigator']['ros__parameters']
    assert navigator['navigators']==['navigate_through_poses']
    assert navigator['robot_base_frame']==changed['runtime']['base_frame'] and navigator['global_frame']==changed['runtime']['map_frame']
    assert navigator['default_nav_through_poses_bt_xml']==str(tmp_path/'config'/'navigate_through_poses.xml')
    (tmp_path/'config'/'navigate_through_poses.xml').unlink()
    with pytest.raises(ValueError,match='Missing behavior tree'):changed.render_nav2('/tmp/map.yaml',tmp_path/'rendered')


@pytest.mark.parametrize('change',[('area_tie_tolerance_ratio',.002),('min_trim_segment_length_m',0),('stripe_spacing_m',-1)])
def test_invalid_settings_rejected(tmp_path,change):
    shutil.copytree(resource('.'),tmp_path/'config',dirs_exist_ok=True)
    path=tmp_path/'config'/'coverage_params.yaml';raw=yaml.safe_load(path.read_text())
    raw['/coverage_config']['ros__parameters']['planning'][change[0]]=change[1]
    path.write_text(yaml.safe_dump(raw))
    with pytest.raises(ValueError):load_settings(tmp_path/'config')


def test_unregistered_maps_cannot_infer_an_offline_start(tmp_path):
    g=room()
    with pytest.raises(ValueError,match='not registered'):offline_case(g)
    path=tmp_path/'cases.yaml';case={'frame':'map','start_xy_m':[1.,1.],'region':'whole_map'}
    config=tmp_path/'config';shutil.copytree(resource('.'),config,dirs_exist_ok=True)
    path=config/'offline_maps.yaml';case['identity']=g.identity
    path.write_text(yaml.safe_dump({'/coverage_preview':{'ros__parameters':{'fixtures':{'map_'+g.identity:case}}}}))
    assert offline_case(g,config)==(case,canonical_hash(case))


def test_global_phase_survives_obstacle_splits_and_origin_rotation():
    g=room();g.cells[13:18,17:21]=100;r=g.reachable(g.world((5,5)),.2)
    for angle in (0.,5.5,90.,math.degrees(g.origin[2])):
        ts=stripes(g,r,r,angle,.1,.35,.2);assert ts
        n=np.array([-math.sin(math.radians(angle)),math.cos(math.radians(angle))])
        for t in ts:
            a=(np.dot(t.start,n)-.1)/.35;b=(np.dot(t.end,n)-.1)/.35
            assert a==pytest.approx(round(a),abs=1e-8) and b==pytest.approx(a,abs=1e-8)
            assert g.segment_safe(t.start,t.end,.2)


def test_boundary_targets_are_safe_polylines_and_do_not_claim_actual_coverage():
    g=room();g.cells[13:18,17:21]=100;s=load_settings();start=g.world((5,5));r=g.reachable(start,s.collision)
    poly=g.world(np.array([[0,0],[0,39],[31,39],[31,0]])).tolist()
    ts,audit=boundary_routes(g,r,poly,start,s);assert ts and audit
    for t in ts:
        assert t.kind=='boundary'
        for a,b in zip(points_of(t),points_of(t)[1:]):assert g.segment_safe(a,b,s.collision)
    from simba_coverage.measurement import Meter
    m=Meter(g,r);assert not m.covered.any()


def test_trim_keeps_phase_and_never_changes_denominator():
    g=room();s=load_settings();r=g.reachable(g.world((5,5)),.2);original=r.copy()
    ts=stripes(g,r,r,0,.1,.35,.2);remaining=r.copy();remaining[:15]=False
    kept,dropped=trim_stripes(g,ts,remaining,s)
    assert np.array_equal(r,original)
    for t in kept:
        assert (t.start[1]-.1)/.35==pytest.approx(round((t.start[1]-.1)/.35),abs=1e-8)
        assert g.segment_safe(t.start,t.end,s.collision)


def test_panel_remaining_consistency_and_rounding():
    from simba_coverage import comparison as module
    den=np.ones((100,100),bool);covered=den.copy();covered[0,0]=False
    a=module.panel_statistics(den,covered,.05)
    assert a['coverage_label']=='<100%' and a['remaining_cells']==1
    assert module.panel_statistics(den,den,.05)['coverage_label']=='100%'
    with pytest.raises(AssertionError):module.panel_statistics(~den,den,.05)


def test_tracked_trapezoidal_fixture_and_wall_evidence():
    path=Path(__file__).parent/'data'/'trapezoid.yaml'
    g=Grid.load(path);s=load_settings();walls=wall_evidence(g,s['planning'])
    assert walls['trusted'] and 'next_cluster_angle_deg' in walls
    assert len(walls['individual_walls'])>=2
    assert g.reachable([2.,1.5],s.collision).any()


def test_real_map_regression_when_deployed():
    path=Path(__file__).resolve().parents[3]/'maps/room_20260923_1338/map.yaml'
    if not path.exists():pytest.skip('SKIP VISIBLE: real map is optional and was not deployed')
    g=Grid.load(path);case,_=offline_case(g);assert case['start_xy_m']==[.807,-1.575]
    walls=wall_evidence(g,load_settings()['planning'])
    assert walls['trusted'] and 'next_cluster_angle_deg' in walls


def test_vector_visibility_matches_scalar_corner_checks():
    from simba_coverage.planning import visible_many
    g=room();g.cells[12:18,18:20]=100;rng=np.random.default_rng(13)
    centres=g.world(np.argwhere(g.free)[rng.integers(0,int(g.free.sum()),size=250)])
    ends=centres+rng.uniform(-.2,.2,centres.shape)
    # Restrict endpoints to free in-map cells, as required by stroke_union.
    keep=[i for i,p in enumerate(ends) if g.valid(g.cell(p)) and g.free[g.cell(p)]]
    a,b=centres[keep],ends[keep]
    assert np.array_equal(visible_many(g,a,b),[g.visible(x,y) for x,y in zip(a,b)])


def test_rotated_rectangular_wall_evidence_and_no_wall_fallback():
    g=room();s=load_settings();w=wall_evidence(g,s['planning'])
    assert w['trusted']
    assert abs((w['dominant_angle_deg']-math.degrees(g.origin[2])+45)%90-45)<1.
    g=Grid.from_cells(np.zeros((5,5),np.int8),.05,(0,0,0))
    assert not wall_evidence(g,s['planning'])['trusted']


def test_selected_region_does_not_become_a_whole_map_sweep():
    g=room();r=g.reachable(g.world((5,5)),.2);den=np.zeros_like(r);den[8:13,8:13]=True
    ts=stripes(g,r,den,math.degrees(g.origin[2]),.1,.35,.2,.25)
    assert ts
    for t in ts:
        for xy in (t.start,t.end):
            y,x=g.cell(xy)
            assert 5<=y<=15 and 5<=x<=15


def test_canonical_hash_numeric_types_do_not_depend_on_yaml_template_spelling():
    assert canonical_hash({'angle':0.,'nested':[1,.12]})==canonical_hash({'nested':[1.,.12],'angle':0})
    assert canonical_hash({'speed':.12})!=canonical_hash({'speed':.120000000000001})


def test_closed_contour_has_distinct_nav2_goal_endpoints():
    from simba_coverage.planning import split_path,length
    ring=[[1.,1.],[1.3,1.],[1.3,1.3],[1.,1.3],[1.,1.]]
    targets=split_path(ring,2.,'boundary')
    assert len(targets)>=2
    assert all(math.dist(t.start,t.end)>.05 for t in targets)
    assert sum(length(points_of(t)) for t in targets)==pytest.approx(length(ring))
    assert all(targets[i].end==targets[i+1].start for i in range(len(targets)-1))


def test_length_splitting_cannot_reintroduce_a_tiny_trimmed_tail():
    from simba_coverage.planning import split_path,length
    ts=split_path([[0.,0.],[2.1,0.]],2.,'sweep')
    assert len(ts)==2
    assert all(.35<=length(points_of(t))<=2. for t in ts)
    assert sum(length(points_of(t)) for t in ts)==pytest.approx(2.1)


def test_standard_preview_renderer_requires_only_pillow(tmp_path):
    from PIL import Image
    from simba_coverage.preview import _draw
    g=room();den=g.free.copy();covered=den.copy();remaining=np.zeros_like(den)
    panel={'covered':covered,'remaining':remaining,'segments':[],
           'stats':{'coverage_label':'100%','path_length_m':0.,'remaining_m2':0.}}
    selected={'angle_deg':0.,'offset_m':0.}
    manifest={'components':{'coverable_mask':'test'}}
    _draw(g,g.world((5,5)),den,panel,tmp_path,selected,manifest)
    with Image.open(tmp_path/'preview.png') as image:
        image.verify()
    assert (tmp_path/'preview.svg').read_text().startswith('<svg ')
