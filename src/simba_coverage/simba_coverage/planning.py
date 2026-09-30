"""Metric equal-phase stripes, wall evidence, boundary routes and deterministic trimming."""
import hashlib
import json
import math
import time
from collections import defaultdict
import numpy as np
from .geometry import wrap
from .planner import target


def length(points):
    a=np.asarray(points,float)
    return float(np.linalg.norm(np.diff(a,axis=0),axis=1).sum()) if len(a)>1 else 0.


def turns(points):
    d=np.diff(np.asarray(points,float),axis=0);d=d[np.linalg.norm(d,axis=1)>1e-8]
    if len(d)<2:return 0.
    a=np.arctan2(d[:,1],d[:,0])
    return float(np.abs(np.arctan2(np.sin(np.diff(a)),np.cos(np.diff(a)))).sum())


def poly_target(points,kind='sweep'):
    points=np.asarray(points,float).tolist()
    t=target(points[0],points[-1]);t.points=points;t.kind=kind
    t.key=hashlib.sha256(json.dumps([kind,points],separators=(',',':')).encode()).hexdigest()[:20]
    return t


def points_of(t):return getattr(t,'points',None) or [t.start,t.end]


def densify(points,spacing):
    """Points at least every `spacing` metres along the polyline, keeping every vertex."""
    out=[list(points[0])]
    for a,b in zip(points,points[1:]):
        n=max(1,math.ceil(math.dist(a,b)/spacing-1e-6))  # Tolerance: densifying twice must not add points.
        out.extend((np.asarray(a,float)+(np.asarray(b,float)-a)*i/n).tolist() for i in range(1,n+1))
    return out


def split_path(points,maximum,kind):
    # Nav2's endpoint goal checker can accept a closed loop without traversing it.
    # Split at a distinct point before applying the ordinary length limit.
    if len(points)>2 and math.dist(points[0],points[-1])<1e-9:
        index=max(range(1,len(points)-1),key=lambda i:math.dist(points[0],points[i]))
        if math.dist(points[0],points[index])<1e-9:return []
        return split_path(points[:index+1],maximum,kind)+split_path(points[index:],maximum,kind)
    total=length(points)
    if total<1e-12:return []
    # Equal arc-length chunks avoid creating a tiny final navigation target.
    maximum=total/max(1,math.ceil(total/maximum))
    result=[];chunk=[list(points[0])];used=0.
    for endpoint in points[1:]:
        endpoint=np.asarray(endpoint,float)
        while math.dist(chunk[-1],endpoint)>maximum-used+1e-9:
            a=np.asarray(chunk[-1]);d=math.dist(a,endpoint)
            q=a+(endpoint-a)*(maximum-used)/d;chunk.append(q.tolist())
            result.append(poly_target(chunk,kind));chunk=[q.tolist()];used=0.
        d=math.dist(chunk[-1],endpoint)
        if d>1e-9:chunk.append(endpoint.tolist());used+=d
    if len(chunk)>1:result.append(poly_target(chunk,kind))
    return result


def wall_evidence(grid,p):
    """Length-weighted line votes; retain individual wall angles before clustering."""
    occupied=grid.cells==100;adj=np.zeros_like(occupied)
    for dy,dx in ((1,0),(-1,0),(0,1),(0,-1)):
        moved=np.roll(grid.free,(dy,dx),(0,1))
        if dy==1:moved[0]=False
        if dy==-1:moved[-1]=False
        if dx==1:moved[:,0]=False
        if dx==-1:moved[:,-1]=False
        adj|=moved
    xy=grid.world(np.argwhere(occupied&adj));proposals=[]
    for deg in np.arange(0,180,p['angle_search_step_deg']):
        a=math.radians(deg);u=np.array([math.cos(a),math.sin(a)]);n=np.array([-u[1],u[0]])
        along=xy@u;rho=xy@n
        bins=np.floor(rho/grid.resolution).astype(int)
        for b in np.unique(bins):
            ids=np.flatnonzero(bins==b);ids=ids[np.argsort(along[ids],kind='stable')]
            for run in np.split(ids,np.flatnonzero(np.diff(along[ids])>3*grid.resolution)+1):
                if len(run)<2:continue
                span=float(np.ptp(along[run]))
                if span>=p['min_wall_length_m']:
                    proposals.append((span,len(run),float(deg),run))
    # Suppress duplicate angular votes for the same physical supporting cells.
    used=set();walls=[]
    for span,_,angle,ids in sorted(proposals,key=lambda x:(-x[0],-x[1],x[2])):
        fresh=set(ids.tolist())-used
        if len(fresh)<.5*len(ids):continue
        used.update(ids.tolist())
        walls.append({'angle_deg':angle,'length_m':span,'support_cells':len(ids),
                      'endpoints':[xy[ids[0]].tolist(),xy[ids[-1]].tolist()]})
    clusters=[]
    for wall in sorted(walls,key=lambda w:(-w['length_m'],w['angle_deg'])):
        angle=wall['angle_deg']%90
        match=next((c for c in clusters if abs((angle-c['angle_deg']+45)%90-45)<=p['wall_cluster_tolerance_deg']),None)
        if match is None:match={'angle_deg':angle,'support_m':0.,'walls':[]};clusters.append(match)
        match['walls'].append(wall);match['support_m']+=wall['length_m']
        angles=np.deg2rad([w['angle_deg']*4 for w in match['walls']]);weights=np.array([w['length_m'] for w in match['walls']])
        match['angle_deg']=float(np.rad2deg(math.atan2(sum(weights*np.sin(angles)),sum(weights*np.cos(angles))))/4%90)
    clusters.sort(key=lambda c:(-c['support_m'],c['angle_deg']));total=sum(c['support_m'] for c in clusters)
    for c in clusters:c['support_ratio']=c['support_m']/total if total else 0.
    good=bool(clusters and clusters[0]['support_m']>=p['min_wall_support_m'] and clusters[0]['support_ratio']>=p['min_wall_support_ratio'])
    a=clusters[0]['angle_deg'] if clusters else None;b=clusters[1]['angle_deg'] if len(clusters)>1 else None
    pair=walls[:1]
    if pair:
        theta=math.radians(pair[0]['angle_deg']);normal=np.array([-math.sin(theta),math.cos(theta)])
        centre=np.mean(pair[0]['endpoints'],axis=0)
        for wall in walls[1:]:
            separation=abs(float((np.mean(wall['endpoints'],axis=0)-centre)@normal))
            if abs((wall['angle_deg']-pair[0]['angle_deg']+90)%180-90)<15 and separation>=1.:
                pair.append(wall);break
    return {'long_wall_pair':pair,'trusted':good,'dominant_angle_deg':a,'next_cluster_angle_deg':b,
            'cluster_mismatch_deg':abs((a-b+45)%90-45) if b is not None else None,
            'clusters':clusters,'individual_walls':walls}


def safe_point(grid,xy,reachable,radius):
    rc=grid.cell(xy)
    return grid.valid(rc) and reachable[rc] and grid.segment_safe(xy,grid.world(rc),radius)


def stripes(grid,reachable,denominator,angle_deg,offset,spacing,collision,coverage_radius=.25):
    a=math.radians(angle_deg);u=np.array([math.cos(a),math.sin(a)]);n=np.array([-u[1],u[0]])
    # Only sweep centres that can service the fixed region, while connectors may
    # use the full reachable component. Never silently expand a selected region.
    relevant=np.zeros_like(reachable);h,w=relevant.shape
    cells=math.ceil(coverage_radius/grid.resolution)
    for dy in range(-cells,cells+1):
        for dx in range(-cells,cells+1):
            if math.hypot(dx,dy)*grid.resolution>coverage_radius+1e-10:continue
            ys=slice(max(0,dy),min(h,h+dy));xs=slice(max(0,dx),min(w,w+dx))
            relevant[ys,xs]|=denominator[max(0,-dy):min(h,h-dy),max(0,-dx):min(w,w-dx)]
    relevant &= reachable
    if not relevant.any():return []
    xy=grid.world(np.argwhere(relevant));lo,hi=np.min(xy@u),np.max(xy@u)
    nlo,nhi=np.min(xy@n),np.max(xy@n);out=[]
    for k in range(math.ceil((nlo-offset)/spacing),math.floor((nhi-offset)/spacing)+1):
        # Metric coordinates preserve phase exactly, even at non-grid-aligned angles.
        values=np.arange(lo,hi+grid.resolution/4,grid.resolution/2)
        points=values[:,None]*u+(offset+k*spacing)*n
        valid=np.array([safe_point(grid,q,relevant,collision) for q in points])
        ids=np.flatnonzero(valid)
        for run in np.split(ids,np.flatnonzero(np.diff(ids)>1)+1):
            if len(run)<2:continue
            part=[points[run[0]].tolist()]
            for i in run[1:]:
                q=points[i].tolist()
                if not grid.segment_safe(part[-1],q,collision):
                    if len(part)>1:out.append((k,poly_target([part[0],part[-1]])))
                    part=[q]
                else:part.append(q)
            if len(part)>1:out.append((k,poly_target([part[0],part[-1]])))
    # Adjacent lanes alternate; obstacle splits retain the traversal order on each lane.
    result=[]
    for index,k in enumerate(sorted(set(k for k,_ in out))):
        group=[t for j,t in out if j==k];group.sort(key=lambda t:np.dot(t.start,u),reverse=bool(index%2))
        for t in group:
            if index%2:t=poly_target([t.end,t.start])
            result.append(t)
    return result


def visible_many(grid, starts, ends):
    """Vectorized exact zero-width segment/closed-cell intersection for short rays."""
    a=grid.local(starts);b=grid.local(ends);v=b-a;res=grid.resolution
    ok=np.ones(len(a),bool)
    if not len(a):return ok
    cells=np.floor(a/res).astype(int);n=math.ceil(float(np.max(np.linalg.norm(v,axis=1)))/res)+1
    h,w=grid.cells.shape
    offsets=np.array([(dx,dy) for dy in range(-n,n+1) for dx in range(-n,n+1)])
    rc=cells[:,None,:]+offsets[None,:,:]
    inside=(rc[:,:,0]>=0)&(rc[:,:,0]<w)&(rc[:,:,1]>=0)&(rc[:,:,1]<h)
    blocked=np.ones(inside.shape,bool)
    blocked[inside]=grid.cells[rc[:,:,1][inside],rc[:,:,0][inside]]!=0
    low=rc*res;high=low+res;tlo=np.zeros(inside.shape);thi=np.ones(inside.shape)
    for j in (0,1):
        aa=a[:,j,None];vv=v[:,j,None];parallel=np.abs(vv)<1e-14
        den=np.where(parallel,1.,vv);t1=(low[:,:,j]-aa)/den;t2=(high[:,:,j]-aa)/den
        tlo=np.maximum(tlo,np.where(parallel,-np.inf,np.minimum(t1,t2)))
        thi=np.minimum(thi,np.where(parallel,np.inf,np.maximum(t1,t2)))
        tlo=np.where(parallel&((aa<low[:,:,j])|(aa>high[:,:,j])),2.,tlo)
    return ~np.any(blocked&(tlo<=thi),axis=1)


def stroke_union(grid,denominator,targets,radius):
    """Ideal visible disk union of complete segments, never actual evidence."""
    mask=np.zeros_like(denominator);rc=np.argwhere(denominator);xy=grid.world(rc)
    for t in targets:
        pts=points_of(t)
        for aa,bb in zip(pts,pts[1:]):
            a,b=np.asarray(aa),np.asarray(bb);v=b-a;vv=float(v@v)
            proj=np.clip((xy-a)@v/vv,0,1) if vv else np.zeros(len(xy))
            q=a+proj[:,None]*v
            near=np.flatnonzero((np.sum((xy-q)**2,axis=1)<=radius**2+1e-12)&~mask[rc[:,0],rc[:,1]])
            good=near[visible_many(grid,q[near],xy[near])]
            mask[rc[good,0],rc[good,1]]=True
    return mask


def connect(grid,targets,start,reachable,collision):
    """Preserve assigned traversal, simplifying only collision-checked connectors."""
    links=[];xy=start
    for t in targets:
        path=[list(xy),list(t.start)] if grid.segment_safe(xy,t.start,collision) else grid.astar(xy,t.start,reachable,collision)
        if not path:raise ValueError('No safe connection to target')
        compact=[path[0]];i=0
        while i<len(path)-1:
            j=i+1;stride=1
            while j<len(path)-1:
                candidate=min(len(path)-1,j+stride)
                if not grid.segment_safe(path[i],path[candidate],collision):break
                j=candidate;stride*=2
            compact.append(path[j]);i=j
        links.append(compact);xy=t.end
    return links


def rank_candidates(candidates,ratio):
    if not 0<=ratio<=.001:raise ValueError('Invalid area tolerance')
    valid=[c for c in candidates if c.get('valid',True)]
    if not valid:raise ValueError('No valid direction/offset candidates')
    maximum=max(c['coverage_cells'] for c in valid);tolerance=math.floor(ratio*maximum)
    eligible=[]
    for c in candidates:
        c.update(global_max_cells=maximum,tolerance_ratio=ratio,tolerance_cells=tolerance,
                 area_deficit_cells=maximum-c.get('coverage_cells',0))
        c['eligible']=c.get('valid',True) and c['area_deficit_cells']<=tolerance
        c['selected']=False;c['selection_basis']=None;c['rank']=None
        c['rejection_reason']=('EFFICIENCY_RANK' if c['eligible'] else 'OUTSIDE_GLOBAL_AREA_BAND') if c.get('valid',True) else c.get('reason','INVALID')
        if c['eligible']:eligible.append(c)
    eligible.sort(key=lambda c:(c['fragment_count'],c['fragment_length_m'],c['estimated_time_s'],c['offset_m'],c['angle_deg']))
    for i,c in enumerate(eligible):c['rank']=i+1
    winner=eligible[0];winner['selected']=True;winner['rejection_reason']=''
    winner['selection_basis']='exact_maximum' if winner['coverage_cells']==maximum else 'within_tolerance'
    return winner


def optimize(grid,reachable,denominator,start,settings,aligned=True,deadline=None):
    p=settings['planning'];wall=wall_evidence(grid,p) if aligned and p['alignment']=='auto' else {'trusted':False,'dominant_angle_deg':None,'next_cluster_angle_deg':None,'clusters':[],'individual_walls':[]}
    axes=[math.degrees(grid.origin[2])%180,(math.degrees(grid.origin[2])+90)%180]
    if aligned:
        a=p['manual_angle_deg'] if p['alignment']=='manual' else wall['dominant_angle_deg'] if wall['trusted'] else None
        if a is not None:axes += [a%180,(a+90)%180]
    axes=sorted(set(round(a,8) for a in axes));records=[];plans={}
    for angle in axes:
        for offset in np.arange(0,p['stripe_spacing_m']-1e-10,(p['offset_step_m'] or grid.resolution)):
            if deadline is not None and time.monotonic()>deadline:raise TimeoutError('Candidate search incomplete')
            record={'angle_deg':angle,'offset_m':float(offset),'valid':False,'coverage_cells':0,'reason':''}
            try:
                ts=stripes(grid,reachable,denominator,angle,offset,p['stripe_spacing_m'],settings.collision,settings['geometry']['coverage_disk_radius_m'])
                if not ts:raise ValueError('No full-length stripes')
                mask=stroke_union(grid,denominator,ts,settings['geometry']['coverage_disk_radius_m'])
                # Candidate ranking excludes the start-to-first-stripe entry leg.
                # This makes direction and phase stable for every start in the same component.
                links=connect(grid,ts,ts[0].start,reachable,settings.collision)
                route=[]
                for t,c in zip(ts,links):route.extend(c+points_of(t))
                record.update(valid=True,coverage_cells=int(mask.sum()),coverage_m2=float(mask.sum()*grid.resolution**2),
                              fragment_count=len(ts),fragment_length_m=sum(length(points_of(t)) for t in ts),
                              connector_length_m=sum(length(c) for c in links),turn_angle_rad=turns(route),
                              estimated_time_s=length(route)/settings['motion']['linear_m_s']+turns(route)/settings['motion']['angular_rad_s'])
                plans[(angle,float(offset))]=(ts,mask)
            except ValueError as e:record['reason']=str(e)
            records.append(record)
    best=rank_candidates(records,p['area_tie_tolerance_ratio']);ts,mask=plans[(best['angle_deg'],best['offset_m'])]
    links=connect(grid,ts,start,reachable,settings.collision)
    return {'targets':ts,'connections':links,'ideal_mask':mask,'candidates':records,'selected':best,'walls':wall,'search_complete':True}


def boundary_routes(grid,reachable,polygon,start,settings):
    """Trace directed safe-cell edges; execute safe owner-cell centres, never raw offsets."""
    region=grid.polygon_mask(polygon);mask=reachable&region;edges={};sources={};outgoing=defaultdict(list)
    for y,x in np.argwhere(mask):
        for dy,dx,a,b in [(-1,0,(x,y),(x+1,y)),(0,1,(x+1,y),(x+1,y+1)),(1,0,(x+1,y+1),(x,y+1)),(0,-1,(x,y+1),(x,y))]:
            yy,xx=y+dy,x+dx
            if not grid.valid((yy,xx)) or not mask[yy,xx]:
                edges[(a,b)]=(int(y),int(x));outgoing[a].append(b)
                if grid.valid((yy,xx)) and reachable[yy,xx] and not region[yy,xx]:
                    sources[(a,b)]='selection_boundary'
                else:
                    n=math.ceil(settings.collision/grid.resolution)+1
                    ys,xs=max(0,y-n),max(0,x-n)
                    cells=grid.cells[ys:y+n+1,xs:x+n+1];blocked=np.argwhere(cells!=0)
                    if len(blocked):
                        d=np.sum((blocked+[ys,xs]-[y,x])**2,axis=1);near=blocked[np.argmin(d)]
                        sources[(a,b)]='unknown_frontier' if cells[tuple(near)]<0 else 'known_obstacle'
                    else:sources[(a,b)]='selection_boundary'

    loops=[];unused=set(edges)
    while unused:
        first=min(unused);edge=first;owners=[];vertices=[];labels=set()
        while edge in unused:
            unused.remove(edge);owners.append(edges[edge]);vertices.append(edge[0]);labels.add(sources[edge]);v=edge[1]
            choices=[b for b in outgoing[v] if (v,b) in unused]
            if not choices:break
            # At a diagonal contact keep the tight left turn; do not join components.
            incoming=np.array(edge[1])-edge[0]
            def turn(b):
                d=np.array(b)-v;return -math.atan2(float(np.cross(incoming,d)),float(incoming@d))
            edge=(v,min(choices,key=lambda b:(turn(b),b)))
        pts=[]
        for rc in owners:
            q=grid.world(rc).tolist()
            if not pts or q!=pts[-1]:pts.append(q)
        if len(pts)<3:continue
        area=sum(a[0]*b[1]-b[0]*a[1] for a,b in zip(vertices,vertices[1:]+vertices[:1]))/2
        if settings['boundary']['scope']=='outer' and area<0:continue
        pts.append(pts[0]);loops.append((pts,'outer' if area>0 else 'obstacle',sorted(labels)))
    targets=[];audit=[];xy=list(start)
    while loops:
        options=[]
        for i,(loop,kind,labels) in enumerate(loops):
            ring=loop[:-1];nearest=min(range(len(ring)),key=lambda j:math.dist(xy,ring[j]))
            for reverse in (False,True):
                side='left' if reverse else 'right'
                if settings['boundary']['direction'] not in ('auto',side):continue
                route=ring[nearest:]+ring[:nearest];route=([route[0]]+list(reversed(route[1:]))) if reverse else route
                route=route+[route[0]];link=grid.astar(xy,route[0],reachable,settings.collision)
                if link:options.append(((length(link)+length(route))/settings['motion']['linear_m_s']+turns(link+route)/settings['motion']['angular_rad_s'],i,side,route,kind,labels))
        if not options:break
        _,i,side,route,kind,labels=min(options,key=lambda v:(v[0],v[1],v[2]));loops.pop(i)
        safe_parts=[];part=[route[0]]
        for q in route[1:]:
            if grid.segment_safe(part[-1],q,settings.collision):part.append(q)
            else:
                if len(part)>1:safe_parts.append(part)
                part=[q]
        if len(part)>1:safe_parts.append(part)
        for part in safe_parts:targets.extend(split_path(part,settings['planning']['max_segment_length_m'],'boundary'))
        if targets:xy=targets[-1].end
        audit.append({'kind':kind,'boundary_sources':labels,'side':side,'closed':len(safe_parts)==1 and safe_parts[0][0]==safe_parts[0][-1],
                      'length_m':sum(length(x) for x in safe_parts),'parts':len(safe_parts)})
    return targets,audit


def merge_intervals(intervals,minimum):
    """Single traversal-order pass. Coordinates increase in assigned traversal order."""
    pending=[list(x) for x in sorted(intervals)];out=[];dropped=[];i=0
    while i<len(pending):
        a,b=pending[i]
        if b-a>=minimum-1e-10:out.append([a,b]);i+=1;continue
        left=bool(out and out[-1][1]<=a and not (dropped and dropped[-1][1]>out[-1][1]))
        right=i+1<len(pending) and pending[i+1][1]-pending[i+1][0]>=minimum-1e-10
        if left or right:
            if right and (not left or pending[i+1][0]-b<=a-out[-1][1]):pending[i+1][0]=a
            else:out[-1][1]=b
            i+=1;continue
        while b-a<minimum-1e-10 and i+1<len(pending):
            i+=1;b=pending[i][1]
        if b-a>=minimum-1e-10:out.append([a,b])
        else:dropped.append([a,b])
        i+=1
    return out,dropped


def trim_stripes(grid,stripe_targets,remaining,settings):
    result=[];residual=[];radius=settings['geometry']['coverage_disk_radius_m']
    for t in stripe_targets:
        a,b=np.asarray(t.start),np.asarray(t.end);distance=float(np.linalg.norm(b-a))
        if distance<1e-9:continue
        u=(b-a)/distance;v=np.linspace(0,distance,max(2,math.ceil(distance/(grid.resolution/2))+1))
        useful=[]
        rc=np.argwhere(remaining);xy=grid.world(rc)
        for d in v:
            q=a+d*u;ids=np.flatnonzero(np.linalg.norm(xy-q,axis=1)<=radius+1e-10)
            useful.append(bool(len(ids) and visible_many(grid,np.tile(q,(len(ids),1)),xy[ids]).any()))
        ids=np.flatnonzero(useful);intervals=[]
        for run in np.split(ids,np.flatnonzero(np.diff(ids)>1)+1):
            if len(run):intervals.append([float(v[run[0]]),float(v[run[-1]])])
        kept,dropped=merge_intervals(intervals,settings['planning']['min_trim_segment_length_m'])
        for lo,hi in kept:
            p,q=(a+lo*u).tolist(),(a+hi*u).tolist()
            if not grid.segment_safe(p,q,settings.collision):dropped.append([lo,hi]);continue
            result.extend(split_path([p,q],settings['planning']['max_segment_length_m'],'sweep'))
        residual.extend({'stripe_key':t.key,'start':(a+lo*u).tolist(),'end':(a+hi*u).tolist(),'reason':'BELOW_MINIMUM_TRIM_LENGTH'} for lo,hi in dropped)
    return result,residual
