#!/usr/bin/env python3
"""Generate an English-labelled, fixed-fixture, four-panel offline comparison."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import numpy as np
from create3_coverage.geometry import Grid
from create3_coverage.settings import load_settings,offline_case
from create3_coverage.planner import gap_targets,ordered
from create3_coverage.planning import (optimize,boundary_routes,trim_stripes,connect,
                                     stroke_union,poly_target,points_of,length,turns)


def mask_hash(mask):return hashlib.sha256(mask.tobytes()).hexdigest()


def panel_statistics(denominator,covered,resolution):
    if np.any(covered&~denominator):raise AssertionError('Coverage outside denominator')
    remaining=denominator&~covered
    assert not np.any(remaining&covered) and np.array_equal(remaining|covered,denominator)
    total=int(denominator.sum());count=int(covered.sum());left=int(remaining.sum())
    if not total:raise ValueError('Empty coverable denominator')
    percent=100*count/total
    label='100%' if left==0 else '<100%' if round(percent,1)==100 else f'{percent:.1f}%'
    assert (label=='100%')==(left==0)
    return {'coverage_cells':count,'remaining_cells':left,'denominator_cells':total,
            'ideal_coverage_ratio':count/total,'coverage_label':label,
            'remaining_m2':left*resolution**2,'covered_m2':count*resolution**2}


def route_segments(targets,connections):
    result=[]
    for t,c in zip(targets,connections):
        result.append({'kind':'connector','points':c})
        result.append({'kind':getattr(t,'kind','sweep'),'points':points_of(t)})
    return result


def union_segments(grid,denominator,segments,radius):
    return stroke_union(grid,denominator,[poly_target(x['points'],x['kind']) for x in segments if len(x['points'])>=2],radius)


def evaluate_panel(name,grid,reachable,denominator,start,settings,segments):
    radius=settings['geometry']['coverage_disk_radius_m'];covered=union_segments(grid,denominator,segments,radius)
    rounds=0;xy=segments[-1]['points'][-1] if segments else start;last_gain=None
    while covered.sum()/denominator.sum()<settings['completion']['target_coverage_ratio'] and settings['resweep']['enabled'] and rounds<settings['resweep']['max_rounds']:
        if last_gain is not None and last_gain<settings['resweep']['minimum_gain_m2']:break
        remaining=denominator&~covered;before=int(covered.sum())
        ts=gap_targets(grid,remaining,reachable,radius,lambda _:False,settings['resweep']['max_targets'])
        ts,links=ordered(grid,ts,xy,reachable,settings.collision)
        if not ts:break
        for t in ts:t.kind='resweep'
        extra=route_segments(ts,links);segments+=extra;covered|=union_segments(grid,denominator,extra,radius)
        xy=ts[-1].end;rounds+=1;last_gain=(int(covered.sum())-before)*grid.resolution**2
    route=[q for seg in segments for q in seg['points']]
    stats=panel_statistics(denominator,covered,grid.resolution)
    stats.update(name=name,path_length_m=sum(length(x['points']) for x in segments),
                 connector_length_m=sum(length(x['points']) for x in segments if x['kind']=='connector'),
                 turn_angle_rad=turns(route),turn_count=int(sum(abs(wrap)>1e-3 for wrap in angle_changes(route))),
                 ideal_resweep_rounds=rounds,actual_coverage_ratio=None)
    stats['estimated_time_s']=stats['path_length_m']/settings['motion']['linear_m_s']+stats['turn_angle_rad']/settings['motion']['angular_rad_s']
    return {'stats':stats,'covered':covered,'remaining':denominator&~covered,'segments':segments}


def angle_changes(points):
    if len(points)<3:return []
    d=np.diff(np.asarray(points),axis=0);d=d[np.linalg.norm(d,axis=1)>1e-8]
    a=np.arctan2(d[:,1],d[:,0]);return np.arctan2(np.sin(np.diff(a)),np.cos(np.diff(a)))


def render(grid,start,denominator,panels,walls,selected,output,spacing):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.colors import ListedColormap
    from matplotlib.collections import LineCollection
    from matplotlib.lines import Line2D
    from matplotlib.patches import Patch
    plt.rcParams.update({'font.family':'DejaVu Sans','font.size':9})
    fig,axes=plt.subplots(1,4,figsize=(20,10),layout='constrained')
    bg=np.where(grid.cells<0,0,np.where(grid.cells==100,1,2));h,w=grid.cells.shape
    extent=[0,w*grid.resolution,0,h*grid.resolution]
    for ax,panel in zip(axes,panels):
        st=panel['stats'];assert panel_statistics(denominator,panel['covered'],grid.resolution)['coverage_label']==st['coverage_label']
        ax.imshow(bg,origin='lower',extent=extent,cmap=ListedColormap(['#bcc2ca','#26313f','#f9fafb']),vmin=0,vmax=2,interpolation='nearest')
        color=np.zeros(bg.shape);color[panel['covered']]=1;color[panel['remaining']]=2
        ax.imshow(np.ma.masked_where(color==0,color),origin='lower',extent=extent,cmap=ListedColormap(['#c3e5d3','#ffd273']),vmin=1,vmax=2,interpolation='nearest',alpha=.75)
        styles={'sweep':('#086994',1.1),'connector':('#d69739',.65),'boundary':('#764ab5',1.2),'resweep':('#d54879',1.3)}
        for kind,(c,lw) in styles.items():
            lines=[grid.local(seg['points']) for seg in panel['segments'] if seg['kind']==kind]
            if lines:ax.add_collection(LineCollection(lines,colors=c,linewidths=lw,alpha=.95))
        # Number complete sweep fragments, not connectors or boundary paths.
        for i,seg in enumerate(x for x in panel['segments'] if x['kind']=='sweep'):
            if i%3==0:
                q=grid.local(seg['points'][0]);ax.text(*q,str(i+1),fontsize=5,color='#064d70')
        if panel is not panels[0]:
            sweeps=[seg['points'] for seg in panel['segments'] if seg['kind']=='sweep']
            marked=False
            for first in sweeps:
                u=np.array(first[-1])-first[0];norm=np.linalg.norm(u)
                if norm<spacing:continue
                u=u/norm;n=np.array([-u[1],u[0]])
                for second in sweeps:
                    distance=abs(float((np.array(second[0])-first[0])@n))
                    if abs(distance-spacing)>1e-7:continue
                    lo=max(min(np.array(first)@u),min(np.array(second)@u));hi=min(max(np.array(first)@u),max(np.array(second)@u))
                    if hi-lo<spacing:continue
                    along=(lo+hi)/2;a=u*along+n*np.dot(first[0],n);b=u*along+n*np.dot(second[0],n)
                    aa,bb=grid.local(a),grid.local(b)
                    ax.annotate('',xy=aa,xytext=bb,arrowprops={'arrowstyle':'<->','color':'#1c324d','lw':1.})
                    mid=(aa+bb)/2;ax.text(mid[0]+.04,mid[1],f'{spacing:g} m',fontsize=7,color='#1c324d',bbox={'facecolor':'white','alpha':.8,'edgecolor':'none'})
                    marked=True;break
                if marked:break
        ax.scatter(*grid.local(start),marker='*',s=65,color='#df493d',zorder=9)
        ax.set_aspect('equal');ax.set_xlim(0,extent[1]);ax.set_ylim(0,extent[3])
        ax.set_xlabel('Map-local x (m)');ax.set_ylabel('Map-local y (m)')
        ax.set_title(f'{st["name"]}\nIdeal {st["coverage_label"]} | {st["path_length_m"]:.1f} m\nRemaining {st["remaining_m2"]:.3f} m²',fontsize=10)
        handles=[Line2D([],[],color=c,label=k.capitalize()) for k,(c,_) in styles.items() if any(s['kind']==k for s in panel['segments'])]
        handles+=[Patch(color='#c3e5d3',label='Ideal covered'),Patch(color='#f9fafb',label='Free outside denominator')]
        if st['remaining_cells']:handles.append(Patch(color='#ffd273',label='Remaining in denominator'))
        ax.legend(handles=handles,loc='lower left',fontsize=6,framealpha=.93)
    next_angle=walls.get('next_cluster_angle_deg');next_text='none' if next_angle is None else f'{next_angle:.2f} deg'
    dominant=walls.get('dominant_angle_deg');dom_text='unavailable' if dominant is None else f'{dominant:.2f} deg'
    fig.suptitle('Create 3 | Fixed-start, fixed-denominator coverage comparison',fontsize=19,fontweight='bold')
    pair=walls.get('long_wall_pair',[])
    side_text=' / '.join(f'{w["angle_deg"]:.2f} deg' for w in pair) or 'unavailable'
    fig.supxlabel(f'Revised main-stripe spacing {spacing:g} m | Selected direction {selected["angle_deg"]:.2f} deg | Dominant wall {dom_text} | Next cluster {next_text}\n'
                  f'Long-wall segment angles: {side_text} | Example start: map ({start[0]:.3f}, {start[1]:.3f}) m | Coverable denominator: {denominator.sum()*grid.resolution**2:.4f} m²\n'
                  'OFFLINE IDEAL GEOMETRY ONLY. No robot, actual pose measurements or docking route. Remaining masks are panel-specific.',fontsize=10)
    fig.savefig(output/'comparison.png',dpi=180);fig.savefig(output/'comparison.svg');plt.close(fig)


def run_preview(map_yaml,output,config=None,registry=None,baseline=None):
    settings=load_settings(config);grid=Grid.load(map_yaml);case,fixture_hash=offline_case(grid,registry)
    start=case['start_xy_m'];reachable=grid.reachable(start,settings.collision)
    h,w=grid.cells.shape;c,s=math.cos(grid.origin[2]),math.sin(grid.origin[2])
    polygon=(np.array([[0,0],[w,0],[w,h],[0,h]])*grid.resolution@np.array([[c,s],[-s,c]])+grid.origin[:2]).tolist()
    if case['region']!='whole_map':polygon=case['region']
    denominator=grid.coverable(reachable,polygon,settings['geometry']['coverage_disk_radius_m'])
    axis=optimize(grid,reachable,denominator,start,settings,aligned=False)
    aligned=optimize(grid,reachable,denominator,start,settings,aligned=True)
    if baseline is None:raise ValueError('Explicit registered baseline required; use modules/coverage/preview.py for single-map preview')
    from create3_coverage.offline import verify_baseline
    verify_baseline(baseline,grid,fixture_hash,settings)
    old=json.loads(Path(baseline).read_text());old_stats=old['summary']
    if old_stats['map_id']!=grid.identity or not np.allclose(old_stats['start_map_xy'],start,atol=1e-10):raise ValueError('Baseline map/start mismatch')
    if not np.allclose(old_stats['region'],polygon) or old_stats['collision_radius_m']!=settings.collision or old_stats['coverage_disk_radius_m']!=settings['geometry']['coverage_disk_radius_m']:raise ValueError('Baseline geometry/region mismatch')
    baseline_segments=[x for x in old['segments'] if x['kind'] in ('connector','sweep')]
    panels=[evaluate_panel('1  Registered baseline',grid,reachable,denominator,start,settings,baseline_segments),
            evaluate_panel('2  Equal spacing',grid,reachable,denominator,start,settings,route_segments(axis['targets'],axis['connections'])),
            evaluate_panel('3  Wall-aligned candidates',grid,reachable,denominator,start,settings,route_segments(aligned['targets'],aligned['connections']))]
    boundary,audit=boundary_routes(grid,reachable,polygon,start,settings) if settings['boundary']['enabled'] else ([],[])
    links=connect(grid,boundary,start,reachable,settings.collision);segments=route_segments(boundary,links)
    painted=union_segments(grid,denominator,segments,settings['geometry']['coverage_disk_radius_m'])
    interior,dropped=trim_stripes(grid,aligned['targets'],denominator&~painted,settings)
    xy=boundary[-1].end if boundary else start;links=connect(grid,interior,xy,reachable,settings.collision)
    segments+=route_segments(interior,links)
    panels.append(evaluate_panel('4  Boundary + interior',grid,reachable,denominator,start,settings,segments))
    out=Path(output);out.mkdir(parents=True,exist_ok=True)
    shared={'map_id':grid.identity,'fixture_hash':fixture_hash,'example_start_not_localized':True,'start_map_xy':start,
            'region':polygon,'reachable_hash':mask_hash(reachable),'denominator_hash':mask_hash(denominator),
            'config_hash':settings.hash,'effective_config':settings.values,'known_free_m2':float(grid.free.sum()*grid.resolution**2),
            'coverable_m2':float(denominator.sum()*grid.resolution**2),'selected':aligned['selected'],'walls':aligned['walls'],
            'boundary_contours':audit,'trim_residual':dropped,'panels':[p['stats'] for p in panels]}
    for p in panels:p['stats'].update(start_map_xy=start,denominator_hash=shared['denominator_hash'],reachable_hash=shared['reachable_hash'],config_hash=settings.hash)
    (out/'comparison.json').write_text(json.dumps(shared,indent=2,allow_nan=False)+'\n')
    (out/'candidates.json').write_text(json.dumps({'equal_spacing':axis['candidates'],'wall_aligned':aligned['candidates']},indent=2,allow_nan=False)+'\n')
    (out/'routes.json').write_text(json.dumps([{'stats':p['stats'],'segments':p['segments']} for p in panels],indent=2)+'\n')
    masks={'reachable':reachable,'denominator':denominator}
    for i,p in enumerate(panels):masks[f'covered_{i}']=p['covered'];masks[f'remaining_{i}']=p['remaining']
    np.savez_compressed(out/'masks.npz',**masks)
    render(grid,start,denominator,panels,aligned['walls'],aligned['selected'],out,settings['planning']['stripe_spacing_m'])
    return shared


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--map',required=True);p.add_argument('--output',required=True)
    p.add_argument('--config');p.add_argument('--registry');p.add_argument('--baseline')
    a=p.parse_args();result=run_preview(a.map,a.output,a.config,a.registry,a.baseline)
    print(json.dumps({'selected':result['selected'],'panels':result['panels']},indent=2))

if __name__=='__main__':main()
