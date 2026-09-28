"""ROS message adapters, timestamp checks, diagnostics and bounded action handling."""
import json
import math
import time
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path, OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan, BatteryState
from std_msgs.msg import String
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from irobot_create_msgs.msg import HazardDetectionVector, WheelStatus, DockStatus
from tf2_ros import Buffer, TransformListener

LATCHED=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
LATEST=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE)


def parameter(node,name,default):
    return node.declare_parameter(name,default).value


def stamp_seconds(stamp):return stamp.sec+stamp.nanosec*1e-9

def yaw(q):return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))

def pose3(p):return [p.position.x,p.position.y,yaw(p.orientation)]

def transform3(t):return [t.transform.translation.x,t.transform.translation.y,yaw(t.transform.rotation)]

def json_msg(data):return String(data=json.dumps(data,allow_nan=False,separators=(',',':')))

def decode(msg):return json.loads(msg.data)

def ros_now(node):return node.get_clock().now().nanoseconds/1e9


def pose_msg(node,xy,heading=0.):
    p=PoseStamped();p.header.frame_id=(getattr(node,'settings',None) or load_settings())['runtime']['map_frame'];p.header.stamp=node.get_clock().now().to_msg()
    p.pose.position.x=float(xy[0]);p.pose.position.y=float(xy[1])
    p.pose.orientation.z=math.sin(heading/2);p.pose.orientation.w=math.cos(heading/2)
    return p


def path_msg(node,points,end_yaw=None):
    msg=Path();msg.header.frame_id=(getattr(node,'settings',None) or load_settings())['runtime']['map_frame'];msg.header.stamp=node.get_clock().now().to_msg()
    for i,p in enumerate(points):
        q=points[min(i+1,len(points)-1)]
        if i==len(points)-1 and i:q=p;p=points[i];old=points[i-1];angle=math.atan2(q[1]-old[1],q[0]-old[0])
        else:angle=math.atan2(q[1]-p[1],q[0]-p[0])
        if i==len(points)-1 and end_yaw is not None:angle=end_yaw
        msg.poses.append(pose_msg(node,p,angle))
    return msg


def grid_msg(node,grid,mask):
    m=OccupancyGrid();m.header.frame_id=(getattr(node,'settings',None) or load_settings())['runtime']['map_frame'];m.header.stamp=node.get_clock().now().to_msg()
    m.info.resolution=grid.resolution;m.info.height,m.info.width=grid.cells.shape
    m.info.origin.position.x=grid.origin[0];m.info.origin.position.y=grid.origin[1]
    m.info.origin.orientation.z=math.sin(grid.origin[2]/2);m.info.origin.orientation.w=math.cos(grid.origin[2]/2)
    m.data=np.where(mask,100,-1).astype(np.int8).ravel().tolist();return m


def diagnostic(node,pub,name,state,values,level=0):
    msg=DiagnosticArray();msg.header.stamp=node.get_clock().now().to_msg()
    d=DiagnosticStatus(name=name,message=state,level=bytes([int(level)]),hardware_id='create3-pi')
    d.values=[KeyValue(key=str(k),value=str(v)) for k,v in values.items()]
    msg.status=[d];pub.publish(msg)


from .params import load_settings


class Inputs:
    def __init__(self,node):
        self.settings=getattr(node,'settings',None) or load_settings()
        self.node=node;self.messages={};self.received={};self.invalid=set()
        for name,typ in [('scan',LaserScan),('odom',Odometry),('battery_state',BatteryState),
                         ('hazard_detection',HazardDetectionVector),('wheel_status',WheelStatus),('dock_status',DockStatus)]:
            node.create_subscription(typ,'/'+name,lambda msg,k=name:self.receive(k,msg),qos_profile_sensor_data)
        self.tf=Buffer(cache_time=Duration(seconds=self.settings['localization']['tf_cache_s']));self.listener=TransformListener(self.tf,node)

    def receive(self,key,msg):
        t=stamp_seconds(msg.header.stamp)
        if key in self.messages and t<stamp_seconds(self.messages[key].header.stamp):self.invalid.add(key)
        else:self.invalid.discard(key)
        self.messages[key]=msg;self.received[key]=time.monotonic()

    def fresh(self,key,limit):
        if key not in self.messages or key in self.invalid:return False
        age=ros_now(self.node)-stamp_seconds(self.messages[key].header.stamp)
        return -self.settings['health']['future_stamp_tolerance_s']<=age<=limit and time.monotonic()-self.received[key]<=limit

    def reason(self,ignore_battery=False):
        for key,limit in [('scan',self.settings['health']['scan_age_s']),('odom',self.settings['health']['odom_age_s']),('hazard_detection',self.settings['health']['hazard_age_s']),('wheel_status',self.settings['health']['wheel_age_s'])]:
            if not self.fresh(key,limit):return key.upper()+'_STALE'
        if not ignore_battery and not self.fresh('battery_state',self.settings['health']['battery_age_s']):return 'BATTERY_STALE'
        if not ignore_battery:
            b=self.messages['battery_state'].percentage
            if not math.isfinite(b) or not 0<=b<=1:return 'BATTERY_INVALID'
            if b<=self.settings['battery']['critical_ratio']:return 'BATTERY_CRITICAL'
        return ''

    def hazards(self):
        msg=self.messages.get('hazard_detection')
        return {h.type for h in msg.detections} if msg else set()

    def stopped(self):
        if not self.fresh('odom',self.settings['health']['odom_age_s']):return False
        t=self.messages['odom'].twist.twist
        return abs(t.linear.x)<self.settings['health']['stop_linear_m_s'] and abs(t.linear.y)<self.settings['health']['stop_linear_m_s'] and abs(t.angular.z)<self.settings['health']['stop_angular_rad_s']

    def lookup(self,target,source,stamp=None):
        return self.tf.lookup_transform(target,source,Time.from_msg(stamp) if stamp else Time())


class ActionSlot:
    """No new action until all old goals, including late accepts, are terminal."""
    def __init__(self,node,mission):
        self.node=node;self.mission=mission;self.clients={};self.records=[];self.events=[]

    def client(self,typ,name):
        if name not in self.clients:self.clients[name]=ActionClient(self.node,typ,name)
        return self.clients[name]

    @property
    def idle(self):return not self.records

    def send(self,typ,name,goal):
        if self.records:raise RuntimeError('Previous action has not terminated')
        client=self.client(typ,name)
        if not client.server_is_ready():raise RuntimeError(name+' server unavailable')
        record={'key':self.mission.action_id(),'name':name,'handle':None,'canceled':False,
                'sent':time.monotonic(),'cancel_at':None}
        self.records.append(record)
        try:future=client.send_goal_async(goal)
        except Exception:
            self.records.remove(record);raise
        future.add_done_callback(lambda f:self.accepted(record,f))

    def finish(self,r,status,result=None,error=''):
        if r in self.records:self.records.remove(r)
        if not r['canceled'] and self.mission.current(r['key']):
            self.events.append((r['name'],status,result,error))

    def accepted(self,r,f):
        try:
            h=f.result();r['handle']=h
            if not h.accepted:self.finish(r,6,error='GOAL_REJECTED');return
            h.get_result_async().add_done_callback(lambda x:self.result(r,x))
            if r['canceled'] or not self.mission.current(r['key']):self.cancel_record(r)
        except Exception as e:self.finish(r,6,error=str(e))

    def result(self,r,f):
        try:
            out=f.result();self.finish(r,out.status,out.result)
        except Exception as e:self.finish(r,6,error=str(e))

    def cancel_record(self,r):
        r['canceled']=True
        if r['cancel_at'] is None:r['cancel_at']=time.monotonic()
        if r['handle'] is not None:
            try:r['handle'].cancel_goal_async()
            except Exception as e:self.node.get_logger().error('Cancel failed: '+str(e))

    def cancel(self):
        self.mission.token+=1;self.events.clear()
        for r in list(self.records):self.cancel_record(r)

    def cancel_overdue(self,now):
        return any(r['cancel_at'] is not None and now-r['cancel_at']>(getattr(self.node,'settings',None) or load_settings())['deadlines']['action_cancel_s'] for r in self.records)

    def acceptance_overdue(self,now):
        return any(r['handle'] is None and now-r['sent']>(getattr(self.node,'settings',None) or load_settings())['deadlines']['action_accept_s'] for r in self.records)
