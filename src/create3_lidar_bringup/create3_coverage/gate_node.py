"""The only external /cmd_vel publisher. Runs locally on the Pi."""
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_srvs.srv import Trigger, SetBool
from diagnostic_msgs.msg import DiagnosticArray
from irobot_create_msgs.srv import EStop
from .health import Ownership
from .settings import node_settings
from .ros_common import Inputs, LATEST, parameter, decode, json_msg, diagnostic


class SafetyGate(Node):
    def __init__(self):
        super().__init__('velocity_safety_gate')
        self.mapping=parameter(self,'mapping_mode',False)
        self.settings=node_settings(self);m=self.settings['motion']
        self.inputs=Inputs(self);self.policy=Ownership(m['linear_m_s'],m['angular_rad_s'],m['linear_accel_m_s2'],m['angular_accel_rad_s2'],self.settings)
        self.last_trust=0.;self.trusted=False;self.peer_hash=''
        self.owns_estop=False;self.estop_pending=None;self.reset_pending=False;self.release_grace=0.
        self.last_lease=0.;self.graph_ok=False;self.graph_reason='GRAPH_NOT_CHECKED'
        self.pub=self.create_publisher(Twist,'/cmd_vel',LATEST)
        self.state_pub=self.create_publisher(String,'/safety/state',LATEST)
        self.diag=self.create_publisher(DiagnosticArray,'/safety/status',10)
        self.estop=self.create_client(EStop,'/e_stop')
        self.create_subscription(Twist,'/cmd_vel_nav',lambda m:self.command('NAV',m),LATEST)
        self.create_subscription(Twist,'/cmd_vel_remote',lambda m:self.command('MANUAL',m),LATEST)
        self.create_subscription(String,'/coverage/lease',self.lease,LATEST)
        self.create_subscription(String,'/coverage/trust',self.trust,LATEST)
        self.create_service(Trigger,'/safety/reset',self.reset)
        self.create_service(Trigger,'/safety/estop',self.stop_service)
        if self.mapping:self.create_service(SetBool,'/control/manual',self.manual)
        self.create_timer(1/self.settings['health']['gate_rate_hz'],self.tick);self.create_timer(self.settings['health']['graph_period_s'],self.check_graph)

    def command(self,source,msg):
        self.policy.receive(source,(msg.linear.x,msg.angular.z),time.monotonic())

    def lease(self,msg):
        if self.mapping:return
        try:
            d=decode(msg)
            self.peer_hash=d.get('config_hash','')
            self.policy.lease(d['owner'],str(d['epoch']),time.monotonic())
            self.last_lease=time.monotonic()
        except (ValueError,KeyError,TypeError):pass

    def trust(self,msg):
        try:self.trusted=bool(decode(msg)['trusted']);self.last_trust=time.monotonic()
        except (ValueError,KeyError,TypeError):self.trusted=False

    def manual(self,req,res):
        if req.data and (self.inputs.reason() or not self.graph_ok or self.policy.fault):
            res.success=False;res.message='Health/graph/reset check required';return res
        self.policy.lease('MANUAL' if req.data else 'NONE',str(time.monotonic()),time.monotonic())
        res.success=True;res.message='Manual ownership changed; fresh commands required';return res

    def check_graph(self):
        info=self.get_publishers_info_by_topic('/cmd_vel')
        ok=len(info)==1 and info[0].node_name==self.get_name() and info[0].node_namespace==self.get_namespace()
        names=[n for n,_ in self.get_node_names_and_namespaces()]
        conflict=('amcl' in names if self.mapping else any('slam_toolbox' in n for n in names))
        self.graph_ok=ok and not conflict
        self.graph_reason='' if self.graph_ok else 'EXTRA_CMD_VEL_PUBLISHER_OR_LOCALIZATION_OWNER'

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

    def reset(self,req,res):
        if self.reset_pending:res.success=False;res.message='EStop release pending';return res
        cause=self.inputs.reason();hazards=self.inputs.hazards()
        if cause or hazards.intersection({2,3,4}) or not self.inputs.stopped() or not self.graph_ok:
            res.success=False;res.message=cause or 'Hazards, motion or graph conflict remain';return res
        if self.policy.owner!='NONE':
            res.success=False;res.message='Pause/cancel or relinquish manual ownership first';return res
        wheels=self.inputs.messages.get('wheel_status')
        if not self.owns_estop and (wheels is None or not wheels.wheels_enabled):
            res.success=False;res.message='Operator/native disabled wheels: restore externally first';return res
        if self.owns_estop:
            if not self.estop.service_is_ready():res.success=False;res.message='EStop service unavailable';return res
            self.reset_pending=True;request=EStop.Request();request.e_stop_on=False
            future=self.estop.call_async(request)
            def done(f):
                try:
                    if f.result().success:self.owns_estop=False;self.policy.fault='';self.release_grace=time.monotonic()+self.settings['health']['reset_grace_s']
                except Exception as e:self.get_logger().error(str(e))
                self.reset_pending=False
            future.add_done_callback(done)
        else:self.policy.fault=''
        res.success=True;res.message='Reset requested; wait for enabled wheels and clear state, then resume'
        return res

    def tick(self):
        now=time.monotonic();hazards=self.inputs.hazards();reason=self.inputs.reason()
        if self.mapping and self.policy.owner=='MANUAL':self.policy.lease_time=now
        if hazards.intersection({2,3,4}):self.assert_estop('SERIOUS_HAZARD_'+','.join(map(str,sorted(hazards.intersection({2,3,4})))))
        wheel=self.inputs.messages.get('wheel_status')
        if wheel and not wheel.wheels_enabled and not self.owns_estop and not self.reset_pending and self.estop_pending is None and now>=self.release_grace:
            self.policy.latch('WHEELS_DISABLED')
        if wheel and not wheel.wheels_enabled:reason=reason or 'WHEELS_DISABLED'
        if not self.graph_ok:reason=reason or self.graph_reason
        if self.policy.owner in ('NAV','NATIVE') and (not self.trusted or now-self.last_trust>self.settings['health']['gate_age_s']):
            reason=reason or 'LOCALIZATION_LOST'
        if not self.mapping and self.policy.owner!='NONE' and now-self.last_lease>self.settings['health']['lease_age_s']:
            reason=reason or 'SUPERVISOR_HEARTBEAT_LOST'
        if not self.mapping and self.policy.owner!='NONE' and self.peer_hash!=self.settings.hash:
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
        if self.policy.fault and self.owns_estop is False and self.policy.fault!='WHEELS_DISABLED':
            self.assert_estop(self.policy.fault)
        state={'config_hash':self.settings.hash,'owner':self.policy.owner,'epoch':self.policy.epoch,'healthy':not reason and not self.policy.fault,
               'reason':reason,'fault':self.policy.fault,'stopped':self.inputs.stopped(),
               'graph_ok':self.graph_ok,'owns_estop':self.owns_estop,'events':self.policy.events}
        self.state_pub.publish(json_msg(state))
        diagnostic(self,self.diag,'velocity_safety_gate',self.policy.fault or reason or 'READY',state,2 if self.policy.fault else int(bool(reason)))


def main(args=None):
    rclpy.init(args=args);node=SafetyGate()
    try:rclpy.spin(node)
    finally:
        # Native ownership must remain silent; external motion gets a finite stop.
        if node.policy.owner in ('NAV','MANUAL'):
            for _ in range(node.settings['health']['shutdown_stop_messages']):node.pub.publish(Twist())
        node.destroy_node();rclpy.shutdown()
