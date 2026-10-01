"""The only external /cmd_vel publisher. Runs locally on the Pi."""
import signal
import time
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Empty as EmptyMsg
from std_srvs.srv import Trigger, SetBool
from diagnostic_msgs.msg import DiagnosticArray
from irobot_create_msgs.srv import EStop
from .health import Ownership
from .params import node_settings
from .ros_common import Inputs, LATEST, parameter, decode, json_msg, diagnostic


class SafetyGate(Node):
    def __init__(self):
        super().__init__('velocity_safety_gate')
        self.mapping=parameter(self,'mapping_mode',False)
        self.supervised_mapping=parameter(self,'supervised_mapping',False)
        if self.supervised_mapping and not self.mapping:
            raise ValueError('supervised_mapping requires mapping_mode')
        self.external_leases=not self.mapping or self.supervised_mapping
        self.settings=node_settings(self);m=self.settings['motion']
        self.inputs=Inputs(self);self.policy=Ownership(m['linear_m_s'],m['angular_rad_s'],m['linear_accel_m_s2'],m['angular_accel_rad_s2'],self.settings)
        self.last_trust=0.;self.trusted=False;self.peer_hash=''
        self.owns_estop=False;self.estop_pending=None;self.reset_pending=False;self.release_grace=0.
        self.last_lease=0.;self.last_manual_heartbeat=0.;self.graph_ok=False;self.graph_reason='GRAPH_NOT_CHECKED';self.graph_conflicts=[]
        self.graph_started=time.monotonic()
        self.pub=self.create_publisher(Twist,'/cmd_vel',LATEST)
        self.state_pub=self.create_publisher(String,'/safety/state',LATEST)
        self.diag=self.create_publisher(DiagnosticArray,'/safety/status',10)
        self.estop=self.create_client(EStop,'/e_stop')
        self.create_subscription(Twist,'/cmd_vel_nav',lambda m:self.command('NAV',m),LATEST)
        self.create_subscription(Twist,'/cmd_vel_remote',lambda m:self.command('MANUAL',m),LATEST)
        self.create_subscription(String,'/coverage/lease',self.lease,LATEST)
        self.create_subscription(String,'/coverage/trust',self.trust,LATEST)
        self.create_subscription(EmptyMsg,'/control/manual_heartbeat',self.manual_heartbeat,10)
        self.create_service(Trigger,'/safety/reset',self.reset)
        self.create_service(Trigger,'/safety/reset_all',self.reset_all)
        self.create_service(Trigger,'/safety/estop',self.stop_service)
        if self.mapping and not self.supervised_mapping:self.create_service(SetBool,'/control/manual',self.manual)
        self.create_timer(1/self.settings['health']['gate_rate_hz'],self.tick);self.create_timer(self.settings['health']['graph_period_s'],self.check_graph)

    def command(self,source,msg):
        self.policy.receive(source,(msg.linear.x,msg.angular.z),time.monotonic())

    def lease(self,msg):
        if not self.external_leases:return
        try:
            d=decode(msg)
            self.peer_hash=d.get('config_hash','')
            self.policy.lease(d['owner'],str(d['epoch']),time.monotonic())
            self.last_lease=time.monotonic()
        except (ValueError,KeyError,TypeError):pass

    def trust(self,msg):
        try:self.trusted=bool(decode(msg)['trusted']);self.last_trust=time.monotonic()
        except (ValueError,KeyError,TypeError):self.trusted=False

    def manual_heartbeat(self,_msg):
        self.last_manual_heartbeat=time.monotonic()
        if self.mapping and not self.supervised_mapping and self.policy.owner=='MANUAL':self.policy.lease_time=self.last_manual_heartbeat

    def manual(self,req,res):
        health_reason=self.inputs.reason()
        if req.data and (health_reason or not self.graph_ok or self.policy.fault):
            blockers=[]
            for blocker in (health_reason, self.graph_reason if not self.graph_ok else '', self.policy.fault):
                if blocker and blocker not in blockers: blockers.append(blocker)
            res.success=False;res.message='Manual ownership denied: '+','.join(blockers);return res
        now=time.monotonic();self.policy.lease('MANUAL' if req.data else 'NONE',str(now),now)
        if req.data:self.last_manual_heartbeat=now
        res.success=True;res.message='Manual ownership changed; fresh commands required';return res

    def check_graph(self):
        publishers=self.get_publishers_info_by_topic('/cmd_vel')
        nodes=self.get_node_names_and_namespaces()
        own=[item for item in publishers if item.node_name==self.get_name() and item.node_namespace==self.get_namespace()]
        matching_nodes=[(name,namespace) for name,namespace in nodes if name==self.get_name()]
        duplicate=[f'{namespace}/{name}' for name,namespace in matching_nodes[1:]]
        extra=[f'{item.node_namespace}/{item.node_name}:{bytes(item.endpoint_gid).hex()}' for item in publishers if item not in own]
        names=[name for name,_ in nodes]
        localization_conflict=('amcl' in names if self.mapping else any('slam_toolbox' in name for name in names))
        coordinator_conflict=self.supervised_mapping and (names.count('exploration_coordinator') != 1 or
            any(name in names for name in ('coverage_supervisor','coverage_meter')) or
            names.count('planner_server') > 1 or names.count('controller_server') > 1 or
            names.count('slam_toolbox') > 1)
        self.graph_conflicts=extra+duplicate+(['LOCALIZATION_OWNER_CONFLICT'] if localization_conflict else [])+(['COORDINATOR_CONFLICT'] if coordinator_conflict else [])
        self.graph_ok=len(publishers)==1 and len(own)==1 and not self.graph_conflicts
        self.graph_reason='' if self.graph_ok else 'CMD_VEL_OR_NODE_OWNERSHIP_CONFLICT'
        if not self.graph_ok and time.monotonic()-self.graph_started>=1.0:self.policy.latch(self.graph_reason)

    def assert_estop(self,reason):
        self.policy.latch(reason)
        if self.estop_pending is not None or self.owns_estop:return
        if not self.estop.service_is_ready():return
        req=EStop.Request();req.e_stop_on=True
        self.estop_pending=self.estop.call_async(req)
        def done(f):
            try:
                if f.result().success:self.owns_estop=True
            except Exception as e:self.get_logger().error(str(e))
            self.estop_pending=None
        self.estop_pending.add_done_callback(done)

    def stop_service(self,req,res):
        self.assert_estop('OPERATOR_OR_ACTION_STOP');res.success=True
        res.message='Stop latched; reset and resume are separate';return res

    def reset_all(self,req,res):
        """Explicit operator recovery, including externally disabled wheels/E-Stop."""
        return self.reset(req,res,restore_wheels=True)

    def reset(self,req,res,restore_wheels=False):
        if self.reset_pending:res.success=False;res.message='EStop release pending';return res
        cause=self.inputs.reason();hazards=self.inputs.hazards()
        if cause or hazards.intersection({2,3,4}) or not self.inputs.stopped() or not self.graph_ok:
            res.success=False;res.message=cause or 'Hazards, motion or graph conflict remain';return res
        if self.policy.owner!='NONE':
            res.success=False;res.message='Pause/cancel or relinquish manual ownership first';return res
        wheels=self.inputs.messages.get('wheel_status')
        if not restore_wheels and not self.owns_estop and (wheels is None or not wheels.wheels_enabled):
            res.success=False;res.message='Operator/native disabled wheels: restore externally first';return res
        if self.owns_estop or restore_wheels:
            if not self.estop.service_is_ready():res.success=False;res.message='EStop service unavailable';return res
            self.reset_pending=True;request=EStop.Request();request.e_stop_on=False
            future=self.estop.call_async(request)
            def done(f):
                try:
                    if f.result().success:self.owns_estop=False;self.policy.fault='';self.release_grace=time.monotonic()+self.settings['health']['reset_grace_s']
                    else:self.get_logger().error('Create 3 refused E-Stop release; safety fault remains')
                except Exception as e:self.get_logger().error(str(e))
                self.reset_pending=False
            future.add_done_callback(done)
        else:self.policy.fault=''
        res.success=True;res.message='Reset requested; wait for enabled wheels and clear state, then resume'
        return res

    def tick(self):
        now=time.monotonic();hazards=self.inputs.hazards();reason=self.inputs.reason()
        if self.mapping and self.policy.owner=='MANUAL' and now-self.last_manual_heartbeat>self.settings['health']['lease_age_s']:
            reason=reason or 'MANUAL_HEARTBEAT_LOST'
        if hazards.intersection({2,3,4}):self.assert_estop('SERIOUS_HAZARD_'+','.join(map(str,sorted(hazards.intersection({2,3,4})))))
        wheel=self.inputs.messages.get('wheel_status')
        if wheel and not wheel.wheels_enabled and not self.owns_estop and not self.reset_pending and self.estop_pending is None and now>=self.release_grace:
            self.policy.latch('WHEELS_DISABLED')
        if wheel and not wheel.wheels_enabled:reason=reason or 'WHEELS_DISABLED'
        if not self.graph_ok:reason=reason or self.graph_reason
        requires_trust=self.policy.owner=='NAV' or (self.policy.owner=='NATIVE' and not self.supervised_mapping)
        if requires_trust and (not self.trusted or now-self.last_trust>self.settings['health']['gate_age_s']):
            reason=reason or 'LOCALIZATION_LOST'
        if self.external_leases and self.policy.owner!='NONE' and now-self.last_lease>self.settings['health']['lease_age_s']:
            reason=reason or 'SUPERVISOR_HEARTBEAT_LOST'
        if self.external_leases and self.policy.owner!='NONE' and self.peer_hash!=self.settings.hash:
            reason=reason or 'CONFIG_HASH_MISMATCH'
        if self.policy.owner=='NATIVE' and (reason or self.policy.fault):
            # Native firmware owns motion: silence alone would not stop it.
            self.assert_estop(reason or self.policy.fault)
        bump=1 in hazards
        if 5 in hazards and self.policy.owner!='NATIVE':reason=reason or 'OBJECT_PROXIMITY'
        if bump and self.policy.owner in ('NAV','MANUAL'):reason=reason or 'BUMP'
        command=self.policy.tick(now,not reason,self.inputs.stopped(),bump=bump,dt=1/self.settings['health']['gate_rate_hz'])
        if command is not None:
            msg=Twist();msg.linear.x,msg.angular.z=command;self.pub.publish(msg)
        if self.policy.stopping and now-self.policy.stop_at>self.settings['deadlines']['action_cancel_s'] and not self.inputs.stopped():
            self.assert_estop('STOP_NOT_CONFIRMED')
        if self.policy.fault and self.owns_estop is False and not self.reset_pending and self.policy.fault!='WHEELS_DISABLED':
            self.assert_estop(self.policy.fault)
        state={'config_hash':self.settings.hash,'owner':self.policy.owner,'epoch':self.policy.epoch,'healthy':not reason and not self.policy.fault,
               'reason':reason,'fault':self.policy.fault,'stopped':self.inputs.stopped(),
               'graph_ok':self.graph_ok,'graph_conflicts':self.graph_conflicts,'owns_estop':self.owns_estop,'events':self.policy.events}
        self.state_pub.publish(json_msg(state))
        diagnostic(self,self.diag,'velocity_safety_gate',self.policy.fault or reason or 'READY',state,2 if self.policy.fault else int(bool(reason)))


def _stop_before_shutdown(node):
    if node.supervised_mapping and node.policy.owner=='NATIVE':
        # Launch may signal all children concurrently. Never rely on the
        # coordinator outliving this gate while a firmware action is active.
        node.assert_estop('GATE_SHUTDOWN_DURING_NATIVE')
        deadline=time.monotonic()+node.settings['health']['stop_confirm_timeout_s']
        while rclpy.ok() and time.monotonic()<deadline and not node.owns_estop:
            rclpy.spin_once(node,timeout_sec=.05)
        return
    if node.policy.owner not in ('NAV','MANUAL') or 1 in node.inputs.hazards():
        return
    started=time.monotonic();sent=0
    while rclpy.ok() and (sent<node.settings['health']['shutdown_stop_messages'] or
                          (not node.inputs.stopped() and time.monotonic()-started<node.settings['health']['stop_confirm_timeout_s'])):
        node.pub.publish(Twist());sent+=1
        rclpy.spin_once(node,timeout_sec=1/node.settings['health']['gate_rate_hz'])
    if not node.inputs.stopped():node.policy.latch('SHUTDOWN_STOP_NOT_CONFIRMED')


def main(args=None):
    rclpy.init(args=args,signal_handler_options=SignalHandlerOptions.NO)
    node=SafetyGate()
    previous={sig:signal.getsignal(sig) for sig in (signal.SIGINT,signal.SIGTERM)}
    stopping=False
    def interrupt(_signum,_frame):
        nonlocal stopping
        stopping=True
    for sig in previous:signal.signal(sig,interrupt)
    try:
        while rclpy.ok() and not stopping:
            rclpy.spin_once(node,timeout_sec=.05)
    finally:
        _stop_before_shutdown(node)
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        for sig,handler in previous.items():signal.signal(sig,handler)


if __name__=='__main__':main()
