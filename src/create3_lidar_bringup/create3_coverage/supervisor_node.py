"""Pi mission supervisor: explicit commands, bounded actions, no automatic launch motion."""
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import json
import math
import time
import numpy as np
import yaml
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import PointStamped, PoseWithCovarianceStamped, PoseStamped
from nav_msgs.msg import OccupancyGrid, Path as PathMsg
from nav2_msgs.action import ComputePathToPose, FollowPath
from irobot_create_msgs.action import Dock, Undock
from std_msgs.msg import String
from std_srvs.srv import Trigger, SetBool, Empty
from diagnostic_msgs.msg import DiagnosticArray
from visualization_msgs.msg import Marker, MarkerArray
from .geometry import Grid, validate_polygon, wrap
from .planner import sweeps, ordered, gap_targets
from .state import Mission
from .settings import node_settings
from .planning import optimize, boundary_routes, connect, points_of, split_path, trim_stripes, length
from .health import Trust
from .ros_common import (Inputs, ActionSlot, LATCHED, LATEST, parameter, decode, json_msg,
                         diagnostic, pose_msg, path_msg, pose3, transform3, yaw, ros_now, stamp_seconds)

ACTIVE={'UNDOCK','CONNECT_PLAN','CONNECT','SWEEP','DWELL','GAP_PLANNING','BUMP_WAIT',
        'PHASE_WAIT','RETURN_PLAN','RETURN','DOCK_WAIT','DOCK','TEST_PLAN','TEST','NATIVE_ARM','DOCK_CONFIRM'}


class Supervisor(Node):
    def __init__(self):
        super().__init__('coverage_supervisor')
        self.settings=node_settings(self);cfg=self.settings
        self.map_frame=cfg['runtime']['map_frame'];self.odom_frame=cfg['runtime']['odom_frame'];self.base_frame=cfg['runtime']['base_frame']
        self.base=Grid.load(cfg['runtime']['map_yaml']);self.grid=self.base
        self.collision=cfg.collision;self.radius=cfg['geometry']['coverage_disk_radius_m']
        self.spacing=cfg['planning']['stripe_spacing_m'];self.timeout=cfg['deadlines']['mission_s']
        self.output=Path(cfg['runtime']['output_dir']);self.dock_file=cfg['runtime']['dock_file']
        self.coverage_phase='MAIN';self.sample_seq=0;self.phase_barrier=0;self.meter_config_hash=''
        self.trim_residual=[];self.phase_checks=[];self.work_previous_phase='MAIN';self.work_previous_round=0;self.work_previous_round_area=0.
        self.inputs=Inputs(self);self.mission=Mission(recovery=self.settings['recovery']);self.slot=ActionSlot(self,self.mission)
        for typ,name in [(ComputePathToPose,'/compute_path_to_pose'),(FollowPath,'/follow_path'),(Dock,'/dock'),(Undock,'/undock')]:
            self.slot.client(typ,name)
        self.trust=Trust(self.settings);self.trusted=False;self.pose=None;self.odom_pose=None;self.amcl=None
        self.live_map=False;self.amcl_received=0.;self.scan_stamp=None;self.scan_score=(0,0)
        self.gate={};self.gate_time=0.;self.measurement={};self.meter_time=0.
        self.region=[];self.editing=False;self.preview=None;self.targets=[];self.current=None
        self.definition=None;self.keepout=[];self.manual=False;self.phase_since=time.monotonic()
        self.native_deadline=0.;self.target_since=0.;self.last_motion=time.monotonic();self.motion_pose=None
        self.return_since=0.;self.round_start_area=0.;self.gain_area=0.;self.no_gain=0.;self.last_tick=time.monotonic()
        self.bump_since=None;self.bump_clear=None;self.bump_locations=[];self.before_bump=''
        self.pending_native=None;self.pending_phase=None;self.path=None;self.path_end_yaw=None;self.test_goal=None
        self.worker_pool=ThreadPoolExecutor(max_workers=1);self.work=None;self.work_kind='';self.work_key=None
        self.dock={};self.load_dock()
        self.lease_pub=self.create_publisher(String,'/coverage/lease',LATEST)
        self.trust_pub=self.create_publisher(String,'/coverage/trust',LATEST)
        self.sample_pub=self.create_publisher(String,'/coverage/sample',LATEST)
        self.definition_pub=self.create_publisher(String,'/coverage/definition',LATCHED)
        self.report_pub=self.create_publisher(String,'/coverage/save_report',LATEST)
        self.state_pub=self.create_publisher(String,'/coverage/state',LATEST)
        self.diag=self.create_publisher(DiagnosticArray,'/coverage/status',10)
        self.route_pub=self.create_publisher(PathMsg,'/coverage/route',LATCHED)
        self.markers=self.create_publisher(MarkerArray,'/coverage/markers',LATCHED)
        self.stop_client=self.create_client(Trigger,'/safety/estop')
        self.nomotion=self.create_client(Empty,'/request_nomotion_update');self.nomotion_pending=False
        self.create_subscription(OccupancyGrid,'/map',self.map_received,LATCHED)
        self.create_subscription(PoseWithCovarianceStamped,'/amcl_pose',self.amcl_pose,10)
        self.create_subscription(String,'/safety/state',self.gate_received,LATEST)
        self.create_subscription(String,'/coverage/measurement',self.meter_received,LATEST)
        self.create_subscription(String,'/coverage/meter_config',self.meter_config_received,LATCHED)
        self.create_subscription(PointStamped,'/coverage/region_point',self.region_point,10)
        self.create_subscription(PoseStamped,'/coverage/test_goal',self.goal_received,10)
        self.create_service(SetBool,'/control/manual',self.manual_service)
        for name,fn in [('region_begin',self.region_begin),('region_commit',self.region_commit),
                        ('region_clear',self.region_clear),('preview',self.preview_service),('start',self.start),
                        ('pause',self.pause_service),('resume',self.resume),('cancel',self.cancel_service),
                        ('return_to_dock',self.return_service),('calibrate_docked',self.calibrate_docked),
                        ('calibrate_staging',self.calibrate_staging),('calibrate_dock_body',self.calibrate_body),
                        ('test_navigation',self.test_navigation)]:
            self.create_service(Trigger,'/coverage/'+name,self.service(fn))
        self.create_timer(1/self.settings['localization']['pose_rate_hz'],self.tick);self.create_timer(self.settings['localization']['nomotion_update_period_s'],self.stationary_update)
        self.create_timer(self.settings['visualization']['period_s'],self.publish_visuals)

    def service(self,fn):
        def call(req,res):
            try:res.message=fn();res.success=True
            except (ValueError,RuntimeError,OSError,KeyError) as e:res.success=False;res.message=str(e)
            return res
        return call

    def idle_for_edit(self):
        if self.mission.state in ACTIVE or self.mission.state in ('PREPARING','PAUSED') or self.manual or not self.slot.idle:
            raise ValueError('Cancel the task/relinquish control before editing configuration')
        if self.work is not None:raise ValueError('Planning is still running')

    def map_received(self,msg):
        try:
            grid=Grid.from_cells(np.asarray(msg.data,np.int8).reshape(msg.info.height,msg.info.width),
                                 msg.info.resolution,(msg.info.origin.position.x,msg.info.origin.position.y,yaw(msg.info.origin.orientation)))
            self.live_map=msg.header.frame_id==self.map_frame and grid.identity==self.base.identity
        except (ValueError,TypeError):self.live_map=False

    def amcl_pose(self,msg):self.amcl=msg;self.amcl_received=time.monotonic()
    def gate_received(self,msg):
        try:self.gate=decode(msg);self.gate_time=time.monotonic()
        except ValueError:pass
    def meter_received(self,msg):
        try:
            d=decode(msg)
            if d['generation']==self.mission.generation:self.measurement=d;self.meter_time=time.monotonic()
        except (ValueError,KeyError):pass

    def meter_config_received(self,msg):
        try:self.meter_config_hash=decode(msg)['config_hash']
        except (ValueError,KeyError):self.meter_config_hash=''

    def config_ready(self):
        return self.gate.get('config_hash')==self.settings.hash and self.meter_config_hash==self.settings.hash

    def update_trust(self,now):
        tf_ok=True;pose=None;odom=None
        try:
            # A short interpolation delay prevents testing against a TF not yet received.
            stamp=Time(nanoseconds=self.get_clock().now().nanoseconds-int(self.settings['localization']['tf_delay_s']*1e9)).to_msg()
            pose=transform3(self.inputs.lookup(self.map_frame,self.base_frame,stamp))
            odom=transform3(self.inputs.lookup(self.odom_frame,self.base_frame,stamp))
            scan=self.inputs.messages.get('scan')
            if scan is not None and self.scan_stamp!=stamp_seconds(scan.header.stamp):
                laser=transform3(self.inputs.lookup(self.map_frame,scan.header.frame_id,scan.header.stamp))
                ranges=np.asarray(scan.ranges);ids=np.flatnonzero(np.isfinite(ranges)&(ranges>=max(self.settings['localization']['min_scan_range_m'],scan.range_min))&(ranges<min(self.settings['localization']['max_scan_range_m'],scan.range_max)))
                if len(ids)>self.settings['localization']['max_endpoints']:ids=ids[np.linspace(0,len(ids)-1,self.settings['localization']['max_endpoints']).astype(int)]
                angles=scan.angle_min+ids*scan.angle_increment+laser[2]
                endpoints=np.column_stack((laser[0]+ranges[ids]*np.cos(angles),laser[1]+ranges[ids]*np.sin(angles)))
                obstacles=self.base.world(np.argwhere(self.base.cells==100))
                matched=0
                for p in endpoints:
                    if len(obstacles) and np.min(np.sum((obstacles-p)**2,axis=1))<=self.settings['localization']['match_distance_m']**2:matched+=1
                self.scan_score=(len(endpoints),matched);self.scan_stamp=stamp_seconds(scan.header.stamp)
        except Exception:tf_ok=False
        self.pose=pose;self.odom_pose=odom
        age=(ros_now(self)-stamp_seconds(self.amcl.header.stamp)) if self.amcl else math.inf
        if now-self.amcl_received>self.settings['localization']['amcl_max_age_s']:age=math.inf
        self.trusted=self.trust.check(now,pose,odom,self.amcl.pose.covariance if self.amcl else None,
                                     *self.scan_score,age,
                                     self.inputs.fresh('scan',self.settings['health']['scan_age_s']) and self.inputs.fresh('odom',self.settings['health']['odom_age_s']) and self.live_map,tf_ok)
        reason=self.trust.reason if self.live_map else 'MAP_IDENTITY_UNVERIFIED'
        self.trust_pub.publish(json_msg({'trusted':self.trusted,'reason':reason}))
        if self.mission.generation:
            self.sample_seq+=1
            self.sample_pub.publish(json_msg({'generation':self.mission.generation,'sample_seq':self.sample_seq,'stamp':ros_now(self)-self.settings['localization']['tf_delay_s'],
                'pose':pose or [0.,0.,0.],'trusted':self.trusted,'active':self.mission.state in ('CONNECT','SWEEP','DWELL','RETURN','PHASE_WAIT') and not self.manual}))

    def stationary_update(self):
        if self.inputs.stopped() and self.nomotion.service_is_ready() and not self.nomotion_pending:
            self.nomotion_pending=True
            f=self.nomotion.call_async(Empty.Request())
            f.add_done_callback(lambda _:setattr(self,'nomotion_pending',False))

    def ready(self,require_stopped=True,require_dock=False):
        now=time.monotonic()
        if self.manual:raise ValueError('Relinquish manual ownership first')
        if not self.trusted or not self.live_map:raise ValueError(self.trust.reason or 'MAP_UNVERIFIED')
        if self.inputs.reason():raise ValueError(self.inputs.reason())
        if now-self.gate_time>self.settings['health']['gate_age_s'] or not self.gate.get('graph_ok') or self.gate.get('fault'):
            raise ValueError('Safety gate unavailable, graph conflict or reset required')
        if not self.inputs.messages['wheel_status'].wheels_enabled:raise ValueError('WHEELS_DISABLED')
        if self.inputs.hazards().intersection({1,2,3,4,5}):raise ValueError('Active hazard prevents start/resume')
        if require_stopped and not self.inputs.stopped():raise ValueError('Robot must be stationary')
        if not self.slot.idle:raise ValueError('Previous action is not yet terminal')
        if require_dock:self.check_dock(validated=True)

    def region_begin(self):
        self.idle_for_edit();self.region=[];self.editing=True;self.preview=None;return 'Click map-frame polygon vertices in RViz'
    def region_point(self,msg):
        if self.editing and msg.header.frame_id==self.map_frame:self.region.append([msg.point.x,msg.point.y])
    def region_commit(self):
        self.idle_for_edit();self.region=validate_polygon(self.region);self.editing=False
        if not np.any(self.base.polygon_mask(self.region)&self.base.free):raise ValueError('Region has no known free floor')
        self.preview=None;return 'Region committed; request preview (no motion)'
    def region_clear(self):
        self.idle_for_edit();self.region=[];self.editing=False;self.preview=None;return 'Region cleared'

    def make_grid(self):
        cells=self.base.cells.copy();self.keepout=[]
        if self.dock.get('body'):
            self.keepout=np.flatnonzero(self.base.polygon_mask(self.dock['body'])).tolist()
            cells.ravel()[self.keepout]=100
        return Grid(cells,self.base.resolution,self.base.origin,self.base.identity)

    def preview_service(self):
        self.idle_for_edit()
        if self.editing or not self.region:raise ValueError('Commit a region first')
        if not self.trusted:raise ValueError('Localize first, or use the separate offline preview tool')
        start=self.pose[:2]
        if self.is_docked():self.check_dock(False);start=self.dock['staging'][:2]
        self.grid=self.make_grid();grid=self.grid;polygon=list(self.region)
        self.preview=None
        def build():
            reachable=grid.reachable(start,self.collision)
            denominator=grid.coverable(reachable,polygon,self.radius)
            if not denominator.any():raise ValueError('No reachable coverable area')
            plan=optimize(grid,reachable,denominator,start,self.settings)
            boundary,audit=boundary_routes(grid,reachable,polygon,start,self.settings) if self.settings['boundary']['enabled'] else ([],[])
            targets=boundary+[part for t in plan['targets'] for part in split_path(points_of(t),self.settings['planning']['max_segment_length_m'],'sweep')]
            connections=connect(grid,targets,start,reachable,self.collision)
            return {'reachable':reachable,'denominator':denominator,'targets':targets,'connections':connections,'start':start,
                    'stripes':plan['targets'],'boundary':boundary,'boundary_audit':audit,'candidates':plan['candidates'],
                    'selected':plan['selected'],'walls':plan['walls'],'config_hash':self.settings.hash}
        self.work=self.worker_pool.submit(build);self.work_kind='preview';self.work_key=None
        return 'Preview calculation queued; inspect /coverage/status and route before start'

    def start(self):
        self.ready(require_dock=True)
        if self.preview is None or self.work is not None:raise ValueError('A completed preview is required')
        if self.preview.get('config_hash')!=self.settings.hash:raise ValueError('PREVIEW_CONFIG_STALE: regenerate preview')
        if not self.config_ready():raise ValueError('CONFIG_HASH_MISMATCH')
        if self.inputs.messages['battery_state'].percentage<self.settings['battery']['start_ratio']:raise ValueError(f"Start requires battery >= {100*self.settings['battery']['start_ratio']:g}%")
        if not self.inputs.fresh('dock_status',self.settings['health']['dock_age_s']):raise ValueError('DOCK_STATUS_STALE')
        if not self.is_docked():self.check_start_component()
        self.mission.start(time.monotonic(),self.timeout)
        self.definition={'generation':self.mission.generation,'map_id':self.base.identity,
                         'config_hash':self.settings.hash,'denominator':np.flatnonzero(self.preview['denominator']).tolist(),'keepout':self.keepout}
        self.definition_pub.publish(json_msg(self.definition));self.measurement={};self.meter_time=0.
        self.coverage_phase='BOUNDARY' if self.settings['boundary']['enabled'] else 'MAIN'
        self.phase_checks=[];self.trim_residual=[]
        self.targets=list(self.preview.get('boundary',[])) if self.coverage_phase=='BOUNDARY' else list(self.preview['targets']);self.current=None;self.pending_phase='UNDOCK' if self.is_docked() else 'NEXT'
        self.phase_since=time.monotonic();self.round_start_area=0.;self.gain_area=0.;self.no_gain=0.
        self.bump_locations=[];self.manual=False
        return 'Task accepted; preparation checks precede motion'

    def check_start_component(self):
        rc=self.grid.cell(self.pose[:2])
        if not self.grid.valid(rc) or not self.preview['reachable'][rc] or not self.grid.segment_safe(self.pose[:2],self.grid.world(rc),self.collision):
            raise ValueError('Actual pose no longer belongs to preview component; cancel and preview again')

    def pause(self,reason):
        if self.mission.state not in ACTIVE and self.mission.state!='PREPARING':return
        if self.mission.state=='GAP_PLANNING':
            self.coverage_phase=self.work_previous_phase;self.mission.rounds=self.work_previous_round;self.round_start_area=self.work_previous_round_area
        self.slot.cancel();self.pending_phase=None;self.mission.pause(reason,time.monotonic())
        self.save_report()

    def pause_service(self):self.pause('OPERATOR_PAUSE');return 'Paused; stopping/cancellation is checked independently'

    def resume(self):
        self.ready()
        if self.work is not None:raise ValueError('Previous planning worker is still finishing')
        if self.mission.state!='PAUSED':raise ValueError('Task is not paused')
        if self.mission.expired(time.monotonic()):raise ValueError('Mission deadline expired; start a new task')
        if not self.mission.coverage_finished:self.check_start_component()
        self.mission.resume()
        if self.mission.coverage_finished:self.pending_phase='RETURN'
        else:
            self.check_start_component()
            if self.current and not self.mission.excluded(self.current):self.targets.insert(0,self.current)
            self.current=None;self.pending_phase='UNDOCK' if self.is_docked() else 'NEXT'
        self.phase_since=time.monotonic();return 'Resume accepted; retained exclusions and original mission deadline'

    def cancel_service(self):
        self.slot.cancel();self.manual=False;self.pending_phase=None
        self.mission.change('CANCELED','OPERATOR_CANCEL',time.monotonic());self.save_report()
        return 'Canceled; no automatic return or retry'

    def manual_service(self,req,res):
        if req.data:
            self.pause('MANUAL_TAKEOVER')
            if not self.slot.idle or not self.inputs.stopped():
                res.success=False;res.message='Pause requested; wait until stopped/action terminal, then retry';return res
            if self.gate.get('fault') or self.inputs.reason():
                res.success=False;res.message='Safety reset/healthy sensors required';return res
        self.manual=req.data;self.mission.token+=1
        res.success=True;res.message='Fresh manual commands required' if req.data else 'Manual released; autonomy needs explicit resume'
        return res

    def is_docked(self):
        return self.inputs.fresh('dock_status',self.settings['health']['dock_age_s']) and self.inputs.messages['dock_status'].is_docked

    def load_dock(self):
        source=Path(self.dock_file) if self.dock_file else self.output/('dock_'+self.base.identity[:16]+'.yaml')
        if source.is_file():
            d=yaml.safe_load(source.read_text())
            if isinstance(d,dict) and d.get('map_id')==self.base.identity:self.dock=d

    def save_dock(self):
        self.dock['map_id']=self.base.identity;self.output.mkdir(parents=True,exist_ok=True)
        p=self.output/('dock_'+self.base.identity[:16]+'.yaml');tmp=p.with_suffix('.yaml.tmp')
        tmp.write_text(yaml.safe_dump(self.dock));tmp.replace(p)

    def check_dock(self,validated):
        if self.dock.get('map_id')!=self.base.identity:raise ValueError('No dock metadata for this map')
        for name in ('docked','staging'):
            p=self.dock.get(name)
            if not isinstance(p,list) or len(p)!=3 or not np.isfinite(p).all():raise ValueError('Missing '+name+' calibration')
        validate_polygon(self.dock.get('body',[]))
        grid=self.make_grid()
        if not grid.segment_safe(self.dock['staging'][:2],self.dock['staging'][:2],self.collision):raise ValueError('Staging pose lacks clearance')
        if validated and not self.dock.get('validated',False):raise ValueError('Explicit return_to_dock calibration trial required first')

    def calibrate_docked(self):
        self.idle_for_edit();self.ready()
        if not self.is_docked():raise ValueError('Fresh docked status required')
        self.dock['docked']=list(self.pose);self.dock['validated']=False;self.save_dock()
        return 'Docked pose recorded; staging/body and explicit docking trial still required'

    def calibrate_staging(self):
        self.idle_for_edit();self.ready()
        if not self.inputs.fresh('dock_status',self.settings['health']['dock_age_s']) or not self.inputs.messages['dock_status'].dock_visible or self.is_docked():
            raise ValueError('Undocked, stationary staging pose with visible dock required')
        self.dock['staging']=list(self.pose);self.dock['validated']=False;self.save_dock();self.preview=None
        return 'Staging position and heading recorded'

    def calibrate_body(self):
        self.idle_for_edit();self.dock['body']=validate_polygon(self.region);self.dock['validated']=False
        self.save_dock();self.preview=None
        return 'Selected polygon saved as dock body; select the coverage region separately'

    def return_service(self):
        self.ready();self.check_dock(False)
        if self.is_docked():return 'Already docked; no action sent'
        if self.mission.state in ACTIVE or self.mission.state=='PREPARING':raise ValueError('Pause/cancel active task first')
        if self.mission.state!='PAUSED':self.mission.start(time.monotonic(),self.timeout)
        self.mission.coverage_finished=True;self.grid=self.make_grid();self.begin_return()
        return 'Explicit return requested (also validates candidate dock calibration on success)'

    def goal_received(self,msg):
        if msg.header.frame_id==self.map_frame:self.test_goal=pose3(msg.pose)

    def test_navigation(self):
        self.idle_for_edit();self.ready()
        if self.test_goal is None or math.dist(self.pose[:2],self.test_goal[:2])>self.settings['recovery']['short_test_max_m']:raise ValueError('Set a map-frame test_goal within 2 metres')
        if self.is_docked():raise ValueError('Short navigation trial starts undocked')
        self.grid=self.make_grid();self.mission.start(time.monotonic(),self.timeout);self.mission.coverage_finished=True
        self.plan_to(self.test_goal[:2],'TEST_PLAN',self.test_goal[2]);self.target_since=time.monotonic()
        return 'Supervised short navigation trial requested; completion stops, without automatic docking'

    def plan_to(self,xy,state,heading=None):
        goal=ComputePathToPose.Goal();goal.goal=pose_msg(self,xy,heading or 0.);goal.planner_id='GridBased';goal.use_start=False
        self.path_end_yaw=heading;self.mission.state=state;self.phase_since=time.monotonic()
        self.slot.send(ComputePathToPose,'/compute_path_to_pose',goal)

    def follow(self,points,state,end_yaw=None):
        if len(points)<2:raise ValueError('Controller path has fewer than two poses')
        for a,b in zip(points,points[1:]):
            if not self.grid.segment_safe(a,b,self.collision):raise ValueError('Unsafe continuous path segment')
        self.path=points;goal=FollowPath.Goal();goal.path=path_msg(self,points,end_yaw)
        goal.controller_id='FollowPath';goal.goal_checker_id='goal_checker';goal.progress_checker_id='progress_checker'
        self.mission.state=state;self.phase_since=time.monotonic();self.motion_pose=list(self.pose);self.last_motion=time.monotonic()
        self.slot.send(FollowPath,'/follow_path',goal)

    def next_target(self):
        self.current=None
        while self.targets:
            t=self.targets.pop(0)
            if not self.mission.excluded(t):self.current=t;break
        if self.current is None:self.finish_pass();return
        self.target_since=time.monotonic()
        self.plan_to(self.current.start,'CONNECT_PLAN')

    def begin_sweep(self):
        t=self.current
        if length(points_of(t))<self.settings['planning']['stationary_target_length_m']:
            self.mission.state='DWELL';self.phase_since=time.monotonic();return
        # Polyline boundary targets retain their contour instead of cutting the chord.
        points=[list(self.pose[:2])]
        for a,b in zip(points_of(t),points_of(t)[1:]):
            n=max(1,math.ceil(math.dist(a,b)/self.grid.resolution))
            points.extend((np.asarray(a)+(np.asarray(b)-a)*i/n).tolist() for i in range(n+1))
        self.follow(points,'SWEEP')

    def finish_pass(self):
        # Require meter acknowledgement of the last issued sample at this phase boundary.
        self.phase_barrier=self.sample_seq+1;self.mission.state='PHASE_WAIT';self.phase_since=time.monotonic()

    def advance_pass(self):
        area=self.measurement.get('covered_m2',0.);fraction=self.measurement.get('fraction',0.)
        remaining=self.preview['denominator'].copy();remaining.ravel()[self.measurement.get('covered_indices',[])]=False
        self.phase_checks.append({'phase':self.coverage_phase,'round':self.mission.rounds,'fraction':fraction,'sample_seq':self.measurement.get('sample_seq',0)})
        self.work_previous_phase=self.coverage_phase;self.work_previous_round=self.mission.rounds;self.work_previous_round_area=self.round_start_area
        if self.coverage_phase=='BOUNDARY':
            self.coverage_phase='MAIN';self.mission.state='GAP_PLANNING'
            def build_main():return trim_stripes(self.grid,self.preview['stripes'],remaining,self.settings)
            self.work=self.worker_pool.submit(build_main);self.work_kind='main'
        else:
            if fraction>=self.settings['completion']['target_coverage_ratio']:
                self.complete_coverage('TARGET_COVERAGE_REACHED');return
            if not self.settings['resweep']['enabled']:
                self.complete_coverage('RESWEEP_DISABLED_BELOW_TARGET');return
            if self.mission.rounds>=self.settings['resweep']['max_rounds']:
                self.complete_coverage('RESWEEP_LIMIT_BELOW_TARGET');return
            if self.mission.rounds>0 and area-self.round_start_area<self.settings['resweep']['minimum_gain_m2']:
                self.complete_coverage('NO_COVERAGE_GAIN_BELOW_TARGET');return
            self.coverage_phase='RESWEEP';self.round_start_area=area;self.mission.rounds+=1;self.mission.state='GAP_PLANNING'
            grid=self.grid;reachable=self.preview['reachable'];start=list(self.pose[:2])
            blacklist=set(self.mission.blacklist);blocked=list(self.mission.blocked)
            def excluded(t):return t.key in blacklist or any(math.dist(t.end,b['xy'])<=b['radius'] for b in blocked)
            def build_gap():
                targets=gap_targets(grid,remaining,reachable,self.radius,excluded,self.settings['resweep']['max_targets'])
                return ordered(grid,targets,start,reachable,self.collision)[0]
            self.work=self.worker_pool.submit(build_gap);self.work_kind='gap'
        self.work_key=(self.mission.generation,self.mission.token);self.phase_since=time.monotonic()

    def complete_coverage(self,reason):
        self.mission.coverage_finished=True;self.mission.reason=reason;self.save_report();self.begin_return()

    def begin_return(self):
        self.return_since=time.monotonic();self.plan_to(self.dock['staging'][:2],'RETURN_PLAN',self.dock['staging'][2])

    def fail(self,reason):
        self.slot.cancel();self.pending_phase=None;self.mission.change('FAILED',reason,time.monotonic());self.save_report()

    def target_failure(self,reason):
        self.slot.cancel()
        if not self.mission.root_cause:self.mission.root_cause=reason
        if self.current:
            count=self.mission.fail_target(self.current,time.monotonic())
            if count<self.settings['recovery']['target_attempts']:self.targets.insert(0,self.current)
            self.mission.events.append({'time':time.monotonic(),'target':self.current.key,'reason':reason})
        self.current=None;self.mission.state='PREPARING';self.pending_phase='NEXT';self.phase_since=time.monotonic()

    def save_report(self):
        if not self.mission.generation:return
        d={'generation':self.mission.generation,'state':self.mission.state,'reason':self.mission.reason,
           'root_cause':self.mission.root_cause,'events':self.mission.events,'blacklist':sorted(self.mission.blacklist),
           'temporary_blockages':self.mission.blocked,'target_failures':self.mission.failures,'resweep_rounds':self.mission.rounds,
           'region':self.region,'collision_radius_m':self.collision,'stripe_spacing_m':self.spacing,
           'elapsed_s':time.monotonic()-self.mission.started,'dock_success':self.mission.state=='FINISHED' and self.is_docked(),
           'coverage_threshold_met':self.measurement.get('fraction',0.)>=self.settings['completion']['target_coverage_ratio'],
           'coverage_phase':self.coverage_phase,'phase_checks':self.phase_checks,'trim_residual':self.trim_residual,
           'config_hash':self.settings.hash,'effective_config':self.settings.values}
        if self.preview:
            roi=self.base.polygon_mask(self.region)&self.base.free
            d['selected_free_m2']=int(roi.sum())*self.base.resolution**2
            d['unreachable_selected_m2']=int((roi&~self.preview['denominator']).sum())*self.base.resolution**2
        self.report_pub.publish(json_msg(d))
        self.output.mkdir(parents=True,exist_ok=True)
        (self.output/('task_'+self.mission.generation+'.json')).write_text(json.dumps(d,indent=2)+'\n')

    def action_result(self,name,status,result,error):
        phase=self.mission.state
        success=status==4 and not error and getattr(result,'error_code',0)==0
        if success and phase in ('CONNECT','SWEEP','RETURN','TEST') and self.path:
            if math.dist(self.pose[:2],self.path[-1])>self.settings['recovery']['endpoint_tolerance_m']:
                success=False;error='ACTION_SUCCEEDED_WITHOUT_REACHING_ENDPOINT'
        if not success:
            if phase in ('CONNECT_PLAN','CONNECT','SWEEP'):self.target_failure(error or 'NAVIGATION_FAILED')
            else:self.pause(error or 'ACTION_FAILED')
            return
        if phase=='UNDOCK':
            if getattr(result,'is_docked',True):self.pause('UNDOCK_RESULT_DOCKED');return
            self.mission.state='PREPARING';self.pending_phase='NEXT';self.phase_since=time.monotonic();self.trust.since=None
        elif phase in ('CONNECT_PLAN','RETURN_PLAN','TEST_PLAN'):
            if result.path.header.frame_id!=self.map_frame:raise ValueError('Planner returned a non-map frame')
            points=[[p.pose.position.x,p.pose.position.y] for p in result.path.poses]
            if len(points)<2:
                # Planner may return one pose for a coincident goal; only accept if already there.
                if points and math.dist(self.pose[:2],points[0])<=self.settings['recovery']['plan_endpoint_tolerance_m']:points=[list(self.pose[:2]),points[0]]
                else:raise ValueError('Empty planner result')
            if math.dist(points[0],self.pose[:2])>self.settings['recovery']['planner_start_tolerance_m']:raise ValueError('Planner start does not match robot')
            expected=(self.current.start if phase=='CONNECT_PLAN' else self.dock['staging'][:2] if phase=='RETURN_PLAN' else self.test_goal[:2])
            if math.dist(points[-1],expected)>self.settings['recovery']['plan_endpoint_tolerance_m']+1e-9:raise ValueError('Planner endpoint misses requested target')
            points.insert(0,list(self.pose[:2]))
            self.follow(points,{'CONNECT_PLAN':'CONNECT','RETURN_PLAN':'RETURN','TEST_PLAN':'TEST'}[phase],self.path_end_yaw)
        elif phase=='CONNECT':self.begin_sweep()
        elif phase=='SWEEP':self.next_target()
        elif phase=='RETURN':self.mission.state='DOCK_WAIT';self.phase_since=time.monotonic()
        elif phase=='DOCK':
            if getattr(result,'is_docked',False):
                self.mission.state='DOCK_CONFIRM';self.phase_since=time.monotonic()
            else:self.pause('DOCK_RESULT_NOT_DOCKED')
        elif phase=='TEST':self.mission.change('FINISHED');self.save_report()

    def request_estop(self):
        if self.stop_client.service_is_ready():self.stop_client.call_async(Trigger.Request())

    def tick(self):
        now=time.monotonic();dt=now-self.last_tick;self.last_tick=now
        self.update_trust(now)
        if self.work is not None and self.work.done():
            work,kind,key=self.work,self.work_kind,self.work_key;self.work=None
            try:
                result=work.result()
                if kind=='preview':
                    self.preview=result;self.mission.reason='PREVIEW_READY'
                    self.output.mkdir(parents=True,exist_ok=True)
                    audit={k:result[k] for k in ('candidates','selected','walls','boundary_audit','config_hash','start')}
                    audit['effective_config']=self.settings.values
                    (self.output/'preview_audit.json').write_text(json.dumps(audit,indent=2)+'\n')
                elif key==(self.mission.generation,self.mission.token) and self.mission.state=='GAP_PLANNING':
                    if kind=='main':self.targets,self.trim_residual=result
                    else:self.targets=result
                    if self.targets:self.next_target()
                    elif kind=='main':self.finish_pass()
                    else:self.complete_coverage('NO_USEFUL_UNCOVERED_TARGETS')
            except Exception as e:
                if kind=='preview':self.mission.reason='PREVIEW_FAILED: '+str(e)
                elif self.mission.state=='GAP_PLANNING':self.pause('GAP_PLAN_FAILED: '+str(e))
        if self.mission.expired(now):self.fail('MISSION_TIMEOUT')
        if self.slot.cancel_overdue(now) and not self.inputs.stopped():self.request_estop()
        if self.slot.acceptance_overdue(now):self.pause('ACTION_ACCEPT_TIMEOUT')
        phase=self.mission.state
        try:
            if phase in ACTIVE:
                fault=self.gate.get('fault') or self.inputs.reason()
                if now-self.gate_time>self.settings['health']['gate_age_s']:fault=fault or 'SAFETY_GATE_STALE'
                if not self.gate.get('graph_ok'):fault=fault or 'VELOCITY_GRAPH_CONFLICT'
                if not self.config_ready():fault=fault or 'CONFIG_HASH_MISMATCH'
                if not self.trusted:fault=fault or self.trust.reason
                if not self.mission.coverage_finished and phase not in ('RETURN_PLAN','RETURN','DOCK_WAIT','DOCK','TEST_PLAN','TEST','DOCK_CONFIRM') and (now-self.meter_time>self.settings['health']['meter_age_s'] or self.measurement.get('fault')):
                    fault=fault or 'COVERAGE_METER_STALE'
                if fault:self.pause(fault)
                elif not self.mission.coverage_finished and phase not in ('RETURN_PLAN','RETURN','DOCK_WAIT','DOCK','TEST_PLAN','TEST','DOCK_CONFIRM') and self.inputs.messages['battery_state'].percentage<=self.settings['battery']['return_ratio']:
                    self.slot.cancel();self.mission.coverage_finished=True;self.mission.state='PREPARING';self.pending_phase='RETURN';self.phase_since=now;self.mission.reason='LOW_BATTERY_RETURN'
                elif phase in ('CONNECT','SWEEP','CONNECT_PLAN') and now-self.target_since>=self.settings['deadlines']['target_s']:self.target_failure('TARGET_TIMEOUT')
                elif phase in ('RETURN_PLAN','RETURN') and now-self.return_since>=self.settings['deadlines']['return_s']:self.pause('RETURN_TIMEOUT')
                elif phase in ('UNDOCK','DOCK') and now>=self.native_deadline:self.pause('NATIVE_ACTION_TIMEOUT')
                elif phase.endswith('_PLAN') and now-self.phase_since>=self.settings['deadlines']['planning_s']:
                    if phase=='CONNECT_PLAN':self.target_failure('PLAN_TIMEOUT')
                    else:self.pause('PLAN_TIMEOUT')
                elif phase=='GAP_PLANNING' and now-self.phase_since>=self.settings['deadlines']['planning_s']:self.pause('GAP_PLAN_TIMEOUT')
                elif phase=='TEST' and now-self.target_since>=self.settings['deadlines']['target_s']:self.pause('TEST_TIMEOUT')
                elif phase in ('CONNECT','SWEEP','RETURN','TEST'):
                    if self.motion_pose is None or math.dist(self.pose[:2],self.motion_pose[:2])>=self.settings['recovery']['movement_translation_m'] or abs(wrap(self.pose[2]-self.motion_pose[2]))>=self.settings['recovery']['movement_rotation_rad']:
                        self.motion_pose=list(self.pose);self.last_motion=now
                    if now-self.last_motion>=self.settings['deadlines']['no_motion_s']:
                        if phase in ('CONNECT','SWEEP'):self.target_failure('NO_MOVEMENT')
                        else:self.pause('NO_MOVEMENT')
                if self.mission.state in ('SWEEP','DWELL'):
                    area=self.measurement.get('covered_m2',0.)
                    if area>self.gain_area:self.gain_area=area;self.no_gain=0.
                    else:self.no_gain+=max(0,dt)
                    if self.no_gain>=self.settings['deadlines']['no_coverage_gain_s']:
                        self.slot.cancel();self.mission.coverage_finished=True;self.mission.state='PREPARING';self.pending_phase='RETURN';self.phase_since=now;self.mission.reason='NO_COVERAGE_PROGRESS'
                self.handle_bump(now)
            # Actions can complete during a pause: token filtering prevents state resurrection.
            while self.slot.events:
                self.action_result(*self.slot.events.pop(0))
            phase=self.mission.state
            if phase=='PREPARING' and now-self.phase_since>self.settings['deadlines']['prepare_s']:self.pause('PREPARATION_TIMEOUT')
            if self.mission.state=='PREPARING' and self.pending_phase and self.slot.idle and self.inputs.stopped():
                if now-self.phase_since>self.settings['deadlines']['prepare_s']:self.pause('PREPARATION_TIMEOUT')
                elif self.trusted and self.gate.get('healthy') and not self.inputs.reason():
                    if self.pending_phase=='RETURN':self.pending_phase=None;self.begin_return()
                    elif self.measurement.get('generation')==self.mission.generation and now-self.meter_time<self.settings['health']['meter_age_s']:
                        next_phase=self.pending_phase;self.pending_phase=None
                        if next_phase=='UNDOCK':
                            self.pending_native='UNDOCK';self.mission.state='NATIVE_ARM';self.phase_since=now
                        else:self.check_start_component();self.next_target()
            elif phase=='PHASE_WAIT':
                if self.measurement.get('last_active_sample_seq',-1)>=self.phase_barrier and self.inputs.stopped():self.advance_pass()
                elif now-self.phase_since>self.settings['health']['meter_age_s']:self.pause('METER_PHASE_ACK_TIMEOUT')
            elif phase=='NATIVE_ARM':
                epoch=self.mission.generation+':'+str(self.mission.token)
                if now-self.phase_since>self.settings['deadlines']['native_ownership_s']:self.pause('NATIVE_OWNERSHIP_TIMEOUT')
                elif self.gate.get('owner')=='NATIVE' and self.gate.get('epoch')==epoch and self.gate.get('healthy') and self.inputs.stopped():
                    phase=self.pending_native;self.mission.state=phase;self.native_deadline=now+(self.settings['deadlines']['undock_s'] if phase=='UNDOCK' else self.settings['deadlines']['dock_s'])
                    if phase=='UNDOCK':self.slot.send(Undock,'/undock',Undock.Goal())
                    else:self.slot.send(Dock,'/dock',Dock.Goal())
            elif phase=='DOCK_CONFIRM':
                if self.is_docked():
                    self.dock['validated']=True;self.save_dock();self.mission.change('FINISHED');self.save_report()
                elif now-self.phase_since>self.settings['deadlines']['dock_confirmation_s']:self.pause('DOCK_STATUS_DISAGREEMENT')
            elif phase=='DWELL' and now-self.phase_since>self.settings['deadlines']['dwell_s']:self.next_target()
            elif phase=='DOCK_WAIT' and self.slot.idle and self.inputs.stopped():
                if now-self.phase_since>self.settings['deadlines']['dock_visibility_s']:self.pause('DOCK_NOT_VISIBLE')
                elif self.inputs.fresh('dock_status',self.settings['health']['dock_age_s']) and self.inputs.messages['dock_status'].dock_visible:
                    self.pending_native='DOCK';self.mission.state='NATIVE_ARM';self.phase_since=now
        except (ValueError,RuntimeError,KeyError) as e:
            self.pause('EXECUTION_CHECK: '+str(e))
        phase=self.mission.state
        owner='MANUAL' if self.manual else ('NATIVE' if phase in ('UNDOCK','DOCK','NATIVE_ARM') else ('NAV' if phase in ('CONNECT','SWEEP','RETURN','TEST') else 'NONE'))
        self.lease_pub.publish(json_msg({'config_hash':self.settings.hash,'owner':owner,'epoch':self.mission.generation+':'+str(self.mission.token)}))
        data={'config_hash':self.settings.hash,'coverage_phase':self.coverage_phase,'generation':self.mission.generation,'state':phase,'reason':self.mission.reason,'root_cause':self.mission.root_cause,
              'trusted':self.trusted,'trust_reason':self.trust.reason,'preview_ready':self.preview is not None,
              'blacklisted_targets':len(self.mission.blacklist),'resweep_rounds':self.mission.rounds,
              'fraction':self.measurement.get('fraction',0.),'owner':owner,'dock_validated':bool(self.dock.get('validated'))}
        self.state_pub.publish(json_msg(data));diagnostic(self,self.diag,'coverage_supervisor',phase,data,int(phase in ('PAUSED','FAILED')))

    def handle_bump(self,now):
        phase=self.mission.state;hazards=self.inputs.hazards()
        if phase in ('UNDOCK','DOCK','NATIVE_ARM','DOCK_CONFIRM'):return  # Native behavior owns its normal bump/proximity handling.
        if 5 in hazards and phase in ('CONNECT','SWEEP','RETURN','TEST'):
            self.pause('OBJECT_PROXIMITY');return
        if 1 in hazards and phase in ('CONNECT','SWEEP','RETURN','TEST','BUMP_WAIT'):
            if phase!='BUMP_WAIT':
                near=sum(math.dist(self.pose[:2],p)<self.settings['recovery']['bump_repeat_radius_m'] for p in self.bump_locations)
                self.bump_locations.append(list(self.pose[:2]))
                if near>=self.settings['recovery']['bump_max_repeats']:self.pause('REPEATED_BUMP');return
                self.before_bump=phase;self.slot.cancel();self.mission.state='BUMP_WAIT';self.bump_since=now
            self.bump_clear=None
            if now-self.bump_since>=self.settings['recovery']['bump_max_s']:self.pause('PERSISTENT_BUMP')
        elif phase=='BUMP_WAIT':
            if self.bump_clear is None:self.bump_clear=now
            if now-self.bump_clear>=self.settings['recovery']['bump_clear_s'] and self.slot.idle and self.inputs.stopped():
                if self.before_bump in ('CONNECT','SWEEP'):self.target_failure('BUMP_REPLAN')
                else:self.pause('BUMP_DURING_RETURN_OR_TEST')

    def publish_visuals(self):
        markers=[]
        def line(points,name,color):
            m=Marker();m.header.frame_id=self.map_frame;m.header.stamp=self.get_clock().now().to_msg()
            m.ns=name;m.id=0;m.type=Marker.LINE_STRIP;m.action=Marker.ADD;m.pose.orientation.w=1.
            m.scale.x=.025;m.color.r,m.color.g,m.color.b=color;m.color.a=1.
            from geometry_msgs.msg import Point
            m.points=[Point(x=float(p[0]),y=float(p[1]),z=.03) for p in points];markers.append(m)
        line(self.region+(self.region[:1] if not self.editing else []),'region',(1.,.7,0.))
        line(self.dock.get('body',[])+self.dock.get('body',[])[:1],'dock_body',(1.,0.,0.))
        if self.current:line([self.current.start,self.current.end],'target',(1.,0.,1.))
        else:line([],'target',(1.,0.,1.))
        text=Marker();text.header.frame_id=self.map_frame;text.ns='status';text.id=0;text.type=Marker.TEXT_VIEW_FACING
        text.pose.orientation.w=1.;text.pose.position.x=self.base.origin[0];text.pose.position.y=self.base.origin[1]
        text.pose.position.z=.3;text.scale.z=.16;text.color.r=text.color.g=text.color.b=text.color.a=1.
        text.text=f'{self.mission.state} {100*self.measurement.get("fraction",0.):.1f}%\n{self.mission.reason or self.trust.reason}'
        markers.append(text);self.markers.publish(MarkerArray(markers=markers))
        if self.preview:
            points=[]
            for t,c in zip(self.preview['targets'],self.preview['connections']):points.extend(c+points_of(t))
            self.route_pub.publish(path_msg(self,points))


def main(args=None):
    rclpy.init(args=args);node=Supervisor()
    try:rclpy.spin(node)
    finally:
        node.slot.cancel();node.manual=False
        node.lease_pub.publish(json_msg({'owner':'NONE','epoch':'shutdown'}))
        node.save_report();node.worker_pool.shutdown(wait=False,cancel_futures=True)
        node.destroy_node();rclpy.shutdown()
