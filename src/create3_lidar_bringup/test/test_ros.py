"""Actual rclpy pub/sub/action tests on loopback in domain 91; never robot inputs."""
import os
os.environ['ROS_DOMAIN_ID']='91'
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE']='LOCALHOST'
os.environ['RMW_IMPLEMENTATION']='rmw_cyclonedds_cpp'
os.environ['CYCLONEDDS_URI']='<CycloneDDS><Domain Id="any"><General><Interfaces><NetworkInterface name="lo" multicast="false"/></Interfaces><AllowMulticast>false</AllowMulticast></General><Discovery><Peers><Peer Address="127.0.0.1"/></Peers></Discovery></Domain></CycloneDDS>'
import math
import time
import threading
import json
import numpy as np
import pytest
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionServer,CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import Twist,TransformStamped,PoseWithCovarianceStamped
from nav_msgs.msg import Odometry,OccupancyGrid
from nav2_msgs.action import FollowPath,ComputePathToPose
from sensor_msgs.msg import LaserScan,BatteryState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage
from irobot_create_msgs.msg import HazardDetectionVector,HazardDetection,WheelStatus,DockStatus
from irobot_create_msgs.srv import EStop
from create3_coverage.ros_common import LATEST,LATCHED,json_msg,ActionSlot,path_msg
from create3_coverage.gate_node import SafetyGate
from create3_coverage.supervisor_node import Supervisor
from create3_coverage.meter_node import CoverageMeter
from create3_coverage.state import Mission
from create3_coverage.settings import node_settings


@pytest.fixture
def ros(tmp_path):
    from PIL import Image
    a=np.full((80,80),254,np.uint8);a[[0,-1],:]=0;a[:,[0,-1]]=0
    Image.fromarray(a).save(tmp_path/'map.pgm')
    (tmp_path/'map.yaml').write_text('image: map.pgm\nresolution: 0.05\norigin: [0, 0, 0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')
    rclpy.init(args=['--ros-args','-p','map_yaml:='+str(tmp_path/'map.yaml'),'-p','output_dir:='+str(tmp_path/'reports')])
    executor=MultiThreadedExecutor(num_threads=4);nodes=[]
    def add(node):nodes.append(node);executor.add_node(node);return node
    def pump(seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:executor.spin_once(timeout_sec=.01)
    yield add,pump,executor,tmp_path
    executor.shutdown()
    for n in nodes:
        if hasattr(n,'worker_pool'):n.worker_pool.shutdown(wait=True,cancel_futures=True)
        executor.remove_node(n);n.destroy_node()
    rclpy.shutdown()


class FakeInputs(Node):
    def __init__(self):
        super().__init__('isolated_fake_robot')
        self.settings=node_settings(self)
        self.hazards=[];self.wheels=True;self.battery=.8;self.x=2.;self.y=2.;self.yaw=0.;self.pose_bad=False
        self.docked=False;self.publish_scan=True;self.battery_enabled=True;self.publish_trust=True;self.request_owner='NONE';self.epoch='test'
        self.outputs=[];self.estops=[];self.native_goals=0;self.final=(0.,0.)
        types={'scan':LaserScan,'odom':Odometry,'battery_state':BatteryState,'hazard_detection':HazardDetectionVector,
               'wheel_status':WheelStatus,'dock_status':DockStatus,'tf':TFMessage,'amcl_pose':PoseWithCovarianceStamped}
        self.pubs={name:self.create_publisher(typ,'/'+name,10) for name,typ in types.items()}
        self.trust=self.create_publisher(String,'/coverage/trust',LATEST)
        self.lease=self.create_publisher(String,'/coverage/lease',LATEST)
        self.nav=self.create_publisher(Twist,'/cmd_vel_nav',LATEST)
        self.map_pub=self.create_publisher(OccupancyGrid,'/map',LATCHED)
        self.create_subscription(Twist,'/cmd_vel',self.command,10)
        self.create_service(EStop,'/e_stop',self.estop)
        self.create_timer(.05,self.tick)
    def command(self,msg):self.outputs.append((time.monotonic(),msg.linear.x,msg.angular.z));self.final=(msg.linear.x,msg.angular.z)
    def estop(self,req,res):self.estops.append(req.e_stop_on);self.wheels=not req.e_stop_on;res.success=True;return res
    def tick(self):
        stamp=self.get_clock().now().to_msg()
        od=Odometry();od.header.stamp=stamp;od.header.frame_id='odom';od.child_frame_id='base_footprint'
        od.pose.pose.position.x=self.x;od.pose.pose.position.y=self.y;od.pose.pose.orientation.w=1.;self.pubs['odom'].publish(od)
        hz=HazardDetectionVector();hz.header.stamp=stamp;hz.detections=[HazardDetection(type=k) for k in self.hazards];self.pubs['hazard_detection'].publish(hz)
        wheel=WheelStatus();wheel.header.stamp=stamp;wheel.wheels_enabled=self.wheels;self.pubs['wheel_status'].publish(wheel)
        if self.battery_enabled:
            bat=BatteryState();bat.header.stamp=stamp;bat.percentage=self.battery;self.pubs['battery_state'].publish(bat)
        dock=DockStatus();dock.header.stamp=stamp;dock.is_docked=self.docked;dock.dock_visible=True;self.pubs['dock_status'].publish(dock)
        if self.publish_scan:
            scan=LaserScan();scan.header.stamp=stamp;scan.header.frame_id='laser';scan.angle_min=-math.pi
            scan.angle_increment=2*math.pi/180;scan.range_min=.1;scan.range_max=12.
            ranges=[]
            for a in scan.angle_min+np.arange(180)*scan.angle_increment:
                c,s=math.cos(a),math.sin(a);dist=[]
                for boundary,coordinate,velocity in [(3.95,self.x,c),(.05,self.x,c),(3.95,self.y,s),(.05,self.y,s)]:
                    if abs(velocity)>1e-9 and (boundary-coordinate)/velocity>0:dist.append((boundary-coordinate)/velocity)
                ranges.append(float(min(dist)))
            scan.ranges=ranges;self.pubs['scan'].publish(scan)
        transforms=[]
        for parent,child,x,y in [('map','odom',0.,0.),('odom','base_footprint',self.x,self.y),('base_footprint','laser',0.,0.)]:
            t=TransformStamped();t.header.stamp=stamp;t.header.frame_id=parent;t.child_frame_id=child
            t.transform.translation.x=x;t.transform.translation.y=y;t.transform.rotation.w=1.;transforms.append(t)
        self.pubs['tf'].publish(TFMessage(transforms=transforms))
        am=PoseWithCovarianceStamped();am.header.stamp=stamp;am.header.frame_id='map';am.pose.pose=od.pose.pose
        am.pose.covariance[0]=am.pose.covariance[7]=.0025 if not self.pose_bad else .04
        am.pose.covariance[35]=math.radians(5)**2;self.pubs['amcl_pose'].publish(am)
        if self.publish_trust:self.trust.publish(json_msg({'trusted':True}))
        if self.request_owner is not None:self.lease.publish(json_msg({'config_hash':self.settings.hash,'owner':self.request_owner,'epoch':self.epoch}))
    def publish_map(self,grid):
        m=OccupancyGrid();m.header.frame_id='map';m.info.resolution=grid.resolution
        m.info.height,m.info.width=grid.cells.shape;m.info.origin.orientation.w=1.;m.data=grid.cells.ravel().tolist();self.map_pub.publish(m)


def test_gate_idle_silence_nav_timeout_native_silence(ros):
    add,pump,_,_=ros;gate=add(SafetyGate());fake=add(FakeInputs());pump(.8)
    assert gate.graph_ok and not fake.outputs
    fake.request_owner='NAV';pump(.2)
    for _ in range(8):fake.nav.publish(Twist(linear=__import__('geometry_msgs.msg',fromlist=['Vector3']).Vector3(x=.3)));pump(.06)
    assert any(v>0 for _,v,_ in fake.outputs)
    pump(.8);assert fake.outputs[-1][1:]==(0.,0.)
    count=len(fake.outputs);pump(.3);assert len(fake.outputs)==count
    fake.request_owner='NATIVE';fake.epoch='native';pump(.3);count=len(fake.outputs);pump(.3)
    assert len(fake.outputs)==count


def test_hazard_classification_and_disabled_wheel_latch(ros):
    add,pump,_,_=ros;gate=add(SafetyGate());fake=add(FakeInputs());pump(.6)
    fake.hazards=[0,1];pump(.2);assert not gate.policy.fault and not fake.estops
    fake.hazards=[2];pump(.3);assert gate.policy.fault.startswith('SERIOUS_HAZARD') and True in fake.estops
    root=gate.policy.fault;fake.hazards=[3];pump(.2);assert gate.policy.fault==root


def test_gate_dropout_extra_publisher_and_low_frequency_battery(ros):
    add,pump,_,_=ros;gate=add(SafetyGate());fake=add(FakeInputs());pump(.7)
    fake.battery_enabled=False;pump(2.2)
    assert gate.inputs.fresh('battery_state',15.) and not gate.inputs.reason()
    fake.publish_scan=False;pump(.6);assert gate.inputs.reason()=='SCAN_STALE'
    rogue=fake.create_publisher(Twist,'/cmd_vel',10);pump(.6)
    assert not gate.graph_ok
    fake.destroy_publisher(rogue)


def test_supervisor_launch_no_motion_and_continuous_trust(ros):
    add,pump,_,_=ros;gate=add(SafetyGate());sup=add(Supervisor());meter=add(CoverageMeter());fake=add(FakeInputs())
    fake.request_owner=None;fake.publish_trust=False;fake.publish_map(sup.base);pump(3.4)
    assert sup.trusted, sup.trust.reason
    assert sup.mission.state=='IDLE' and gate.policy.owner=='NONE' and not fake.outputs
    fake.pose_bad=True;pump(.3)
    assert not sup.trusted and sup.trust.reason=='POSITION_UNCERTAIN'
    assert not meter.generation


def test_fake_action_late_result_cannot_mutate_new_generation(ros):
    add,pump,executor,_=ros;server_node=add(Node('fake_action_server'));client=add(Node('action_client_test'));m=Mission();m.start(time.monotonic());slot=ActionSlot(client,m)
    entered=threading.Event();release=threading.Event()
    def execute(handle):
        entered.set();release.wait(2.)
        if handle.is_cancel_requested:handle.canceled()
        else:handle.succeed()
        return FollowPath.Result()
    server=ActionServer(server_node,FollowPath,'/follow_path',execute_callback=execute,
                        cancel_callback=lambda _:CancelResponse.ACCEPT,callback_group=ReentrantCallbackGroup())
    pump(.4)
    goal=FollowPath.Goal();goal.path=path_msg(client,[[1.,1.],[1.1,1.]])
    slot.send(FollowPath,'/follow_path',goal);pump(.3);assert entered.is_set()
    slot.cancel();m.pause('OPERATOR',time.monotonic());m.change('CANCELED');m.start(time.monotonic())
    release.set();pump(.5)
    assert slot.idle and not slot.events and m.state=='PREPARING'
    server.destroy()


def test_pause_preserves_target_exclusions_and_total_watchdog(ros):
    add,pump,_,_=ros;gate=add(SafetyGate());sup=add(Supervisor());fake=add(FakeInputs());fake.request_owner=None;fake.publish_trust=False
    fake.publish_map(sup.base);pump(3.)
    sup.mission.start(time.monotonic()-1801);sup.mission.state='PAUSED';sup.mission.blacklist.add('target-x')
    sup.mission.blocked.append({'xy':[1,1],'radius':.25});pump(.3)
    assert sup.mission.state=='FAILED' and sup.mission.reason=='MISSION_TIMEOUT'
    assert 'target-x' in sup.mission.blacklist and not fake.outputs


def test_small_task_fake_actions_and_native_ownership_handshake(ros):
    from create3_coverage.planner import target
    from irobot_create_msgs.action import Dock
    add,pump,_,_=ros
    gate=add(SafetyGate());sup=add(Supervisor());meter=add(CoverageMeter());fake=add(FakeInputs())
    fake.request_owner=None;fake.publish_trust=False;fake.publish_map(sup.base)
    confirmations=[]
    def plan(handle):
        result=ComputePathToPose.Result();result.path=path_msg(fake,[[fake.x,fake.y],[handle.request.goal.pose.position.x,handle.request.goal.pose.position.y]])
        handle.succeed();return result
    def follow(handle):
        handle.succeed();return FollowPath.Result()
    def dock(handle):
        confirmations.append((gate.policy.owner,gate.policy.epoch))
        fake.docked=True;handle.succeed();return Dock.Result(is_docked=True)
    servers=[ActionServer(fake,ComputePathToPose,'/compute_path_to_pose',execute_callback=plan,callback_group=ReentrantCallbackGroup()),
             ActionServer(fake,FollowPath,'/follow_path',execute_callback=follow,callback_group=ReentrantCallbackGroup()),
             ActionServer(fake,Dock,'/dock',execute_callback=dock,callback_group=ReentrantCallbackGroup())]
    pump(3.2);assert sup.trusted,sup.trust.reason
    sup.region=[[1.85,1.85],[2.15,1.85],[2.15,2.15],[1.85,2.15]]
    sup.dock={'map_id':sup.base.identity,'docked':[2.,2.,0.],'staging':[2.,2.,0.],
              'body':[[3.6,3.6],[3.8,3.6],[3.8,3.8],[3.6,3.8]],'validated':True}
    sup.grid=sup.make_grid();reach=sup.grid.safe_centres(.20);den=sup.grid.polygon_mask(sup.region)
    sup.preview={'reachable':reach,'denominator':den,'targets':[target([2.,2.],[2.,2.])],
                 'connections':[[[2.,2.],[2.,2.]]],'start':[2.,2.],'config_hash':sup.settings.hash,
                 'boundary':[target([2.,2.],[2.,2.])],'stripes':[target([2.,2.],[2.,2.])]}
    sup.start();pump(4.)
    assert sup.mission.state=='FINISHED',(sup.mission.state,sup.mission.reason,sup.trust.reason)
    assert confirmations and confirmations[0][0]=='NATIVE'
    assert meter.meter.report()['fraction']==1.
    assert sup.mission.coverage_finished
    for server in servers:server.destroy()


def test_meter_rejects_planned_path_and_untrusted_samples(ros):
    add,pump,_,_=ros;meter=add(CoverageMeter());fake=add(Node('meter_test_client'))
    defs=fake.create_publisher(String,'/coverage/definition',LATCHED)
    samples=fake.create_publisher(String,'/coverage/sample',LATEST)
    routes=fake.create_publisher(__import__('nav_msgs.msg',fromlist=['Path']).Path,'/coverage/route',LATCHED)
    defs.publish(json_msg({'generation':'g','config_hash':meter.settings.hash,'map_id':meter.base.identity,'denominator':np.flatnonzero(meter.base.free).tolist(),'keepout':[]}));pump(.3)
    routes.publish(path_msg(fake,[[1.,1.],[3.,3.]]));pump(.2)
    assert not meter.meter.covered.any()
    samples.publish(json_msg({'generation':'g','stamp':fake.get_clock().now().nanoseconds/1e9,'pose':[1.,1.,0.],'trusted':False,'active':True}));pump(.2)
    assert not meter.meter.covered.any()
    samples.publish(json_msg({'generation':'g','stamp':fake.get_clock().now().nanoseconds/1e9,'pose':[1.,1.,0.],'trusted':True,'active':True}));pump(.2)
    assert meter.meter.covered.any()


def test_idle_mapping_requires_explicit_manual_grant(ros):
    from rcl_interfaces.msg import ParameterValue
    # Core gate defaults NONE; no keyboard message alone can claim ownership.
    add,pump,_,_=ros;gate=add(SafetyGate());fake=add(FakeInputs());pump(.7)
    keyboard=fake.create_publisher(Twist,'/cmd_vel_remote',LATEST)
    msg=Twist();msg.linear.x=.25
    for _ in range(4):keyboard.publish(msg);pump(.1)
    assert not fake.outputs and gate.policy.owner=='NONE'


def test_matching_nodes_still_reject_stale_preview(ros):
    add,_,_,_=ros;sup=add(Supervisor())
    sup.ready=lambda **_:None
    sup.gate={'config_hash':sup.settings.hash};sup.meter_config_hash=sup.settings.hash
    sup.preview={'config_hash':'reviewed-older-config'}
    with pytest.raises(ValueError,match='PREVIEW_CONFIG_STALE'):sup.start()
    assert sup.mission.state=='IDLE'


def test_boundary_threshold_never_skips_main_and_resweep_switch(ros):
    add,_,_,_=ros;sup=add(Supervisor())
    sup.mission.start(time.monotonic());sup.preview={'denominator':np.ones(sup.grid.cells.shape,bool),'stripes':[]}
    sup.measurement={'covered_m2':1.,'fraction':.95,'covered_indices':[],'sample_seq':3}
    sup.coverage_phase='BOUNDARY';calls=[];sup.complete_coverage=calls.append
    sup.advance_pass();assert sup.coverage_phase=='MAIN' and not calls
    sup.work.result();sup.work=None
    sup.advance_pass();assert calls==['TARGET_COVERAGE_REACHED']
    calls.clear();sup.measurement['fraction']=.80;sup.settings.values['resweep']['enabled']=False
    sup.advance_pass();assert calls==['RESWEEP_DISABLED_BELOW_TARGET']


def test_pause_during_trim_preserves_need_to_build_main(ros):
    add,_,_,_=ros;sup=add(Supervisor())
    sup.mission.start(time.monotonic());sup.mission.state='GAP_PLANNING'
    sup.coverage_phase='MAIN';sup.work_previous_phase='BOUNDARY';sup.work_previous_round=0
    sup.pause('OPERATOR_PAUSE')
    assert sup.coverage_phase=='BOUNDARY' and sup.mission.state=='PAUSED'


def test_native_config_mismatch_triggers_fake_stop(ros):
    add,pump,_,_=ros;gate=add(SafetyGate());fake=add(FakeInputs());pump(.7)
    fake.request_owner=None;gate.policy.lease('NATIVE','native-test',time.monotonic())
    gate.last_lease=time.monotonic();gate.peer_hash='wrong-config';pump(.15)
    assert gate.policy.fault=='CONFIG_HASH_MISMATCH' and True in fake.estops


def test_closed_boundary_loop_is_followed_not_treated_as_a_point(ros):
    from create3_coverage.planning import poly_target
    add,_,_,_=ros;sup=add(Supervisor());sup.pose=[2.,2.,0.]
    sup.current=poly_target([[2.,2.],[2.2,2.],[2.2,2.2],[2.,2.]],'boundary')
    calls=[];sup.follow=lambda points,state:calls.append((points,state))
    sup.begin_sweep()
    assert calls and calls[0][1]=='SWEEP' and len(calls[0][0])>3


def test_new_meter_generation_discards_previous_phase_acknowledgement(ros):
    add,_,_,_=ros;meter=add(CoverageMeter());meter.sample_seq=500;meter.last_active_sample_seq=500
    meter.definition(json_msg({'generation':'new','config_hash':meter.settings.hash,'map_id':meter.base.identity,
                               'denominator':np.flatnonzero(meter.base.free).tolist(),'keepout':[]}))
    assert meter.generation=='new' and meter.last_active_sample_seq==0 and meter.sample_seq==0
