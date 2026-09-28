"""Independent Pi node for actual trusted map-frame pose coverage."""
import json
import time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from nav_msgs.msg import OccupancyGrid, Path as PathMsg
from .geometry import Grid
from .params import node_settings
from .measurement import Meter
from .ros_common import LATCHED,LATEST,parameter,decode,json_msg,grid_msg,path_msg,ros_now


class CoverageMeter(Node):
    def __init__(self):
        super().__init__('coverage_meter')
        self.settings=node_settings(self)
        self.base=Grid.load(self.settings['runtime']['map_yaml'])
        self.radius=self.settings['geometry']['coverage_disk_radius_m']
        self.linear=self.settings['motion']['linear_m_s'];self.angular=self.settings['motion']['angular_rad_s']
        self.output=Path(self.settings['runtime']['output_dir']);self.sample_seq=0;self.last_active_sample_seq=0
        self.config_pub=self.create_publisher(String,'/coverage/meter_config',LATCHED)
        self.config_pub.publish(json_msg({'config_hash':self.settings.hash}))
        self.meter=None;self.generation='';self.fault='';self.last_sample=0.
        self.status=self.create_publisher(String,'/coverage/measurement',LATEST)
        self.covered=self.create_publisher(OccupancyGrid,'/coverage/covered',LATCHED)
        self.remaining=self.create_publisher(OccupancyGrid,'/coverage/remaining',LATCHED)
        self.trajectory=self.create_publisher(PathMsg,'/coverage/trajectory',LATCHED)
        self.trace_segments=self.create_publisher(Marker,'/coverage/trajectory_segments',LATCHED)
        self.create_subscription(String,'/coverage/definition',self.definition,LATCHED)
        self.create_subscription(String,'/coverage/sample',self.sample,LATEST)
        self.create_subscription(String,'/coverage/save_report',self.save,LATEST)
        self.create_timer(self.settings['measurement']['publish_period_s'],self.publish)

    def definition(self,msg):
        try:
            d=decode(msg)
            if d['generation']==self.generation:return
            if d.get('config_hash')!=self.settings.hash:raise ValueError('Config identity mismatch')
            if d['map_id']!=self.base.identity:raise ValueError('Map identity mismatch')
            mask=np.zeros(self.base.cells.size,bool);mask[d['denominator']]=True
            cells=self.base.cells.copy();cells.ravel()[d.get('keepout',[])]=100
            grid=Grid(cells,self.base.resolution,self.base.origin,self.base.identity)
            if not mask.any() or np.any(mask.reshape(cells.shape)&~grid.free):raise ValueError('Invalid denominator')
            self.meter=Meter(grid,mask.reshape(cells.shape),self.radius,max_gap=self.settings['measurement']['max_sample_gap_s'],linear=self.linear,angular=self.angular,translation_slack=self.settings['measurement']['translation_slack_m'],rotation_slack=self.settings['measurement']['rotation_slack_deg'])
            self.generation=d['generation'];self.fault='';self.sample_seq=0;self.last_active_sample_seq=0;self.last_sample=0.
        except (ValueError,KeyError,IndexError,TypeError) as e:self.fault=str(e)

    def sample(self,msg):
        if self.meter is None:return
        try:
            d=decode(msg)
            if d['generation']!=self.generation:return
            age=ros_now(self)-d['stamp']
            trusted=d['trusted'] and 0<=age<=self.settings['measurement']['max_sample_gap_s']
            self.meter.sample(d['pose'],d['stamp'],trusted,d['active'])
            self.last_sample=time.monotonic();self.sample_seq=d.get('sample_seq',0)
            if trusted and d['active'] and self.meter.previous is not None:
                self.last_active_sample_seq=self.sample_seq
        except (ValueError,KeyError,TypeError) as e:self.fault=str(e);self.meter.previous=None

    def publish(self):
        if self.meter is None:return
        if time.monotonic()-self.last_sample>self.settings['measurement']['max_sample_gap_s']:self.meter.previous=None
        data={**self.meter.report(),'config_hash':self.settings.hash,'sample_seq':self.sample_seq,'last_active_sample_seq':self.last_active_sample_seq,'generation':self.generation,'map_id':self.base.identity,
              'known_free_m2':float(self.base.free.sum()*self.base.resolution**2),
              'fault':self.fault,'covered_indices':np.flatnonzero(self.meter.covered).tolist()}
        self.status.publish(json_msg(data))
        self.covered.publish(grid_msg(self,self.meter.grid,self.meter.covered))
        self.remaining.publish(grid_msg(self,self.meter.grid,self.meter.denominator&~self.meter.covered))
        # Never draw a line through a sampling gap in the trajectory display.
        # A LINE_LIST retains all verified edges without joining separate trace segments.
        marker=Marker();marker.header.frame_id=self.settings['runtime']['map_frame'];marker.header.stamp=self.get_clock().now().to_msg()
        marker.ns='actual_trace';marker.id=0;marker.type=Marker.LINE_LIST;marker.action=Marker.ADD
        marker.pose.orientation.w=1.;marker.scale.x=.025
        marker.color.r=1.;marker.color.g=.35;marker.color.a=1.
        for edge in self.meter.trajectory_edges:
            for x,y in edge:marker.points.append(Point(x=float(x),y=float(y),z=.04))
        self.trace_segments.publish(marker)
        trace=self.meter.trajectory
        begin=self.meter.trajectory_segment_start
        self.trajectory.publish(path_msg(self,[x[1:3] for x in trace[max(0,begin):]]))

    def save(self,msg):
        if self.meter is None:return
        try:
            d=decode(msg)
            if d['generation']!=self.generation:return
            self.output.mkdir(parents=True,exist_ok=True)
            dest=self.output/('coverage_'+self.generation+'.json')
            data={**d,**self.meter.report(),'map_id':self.base.identity,'coverage_disk_radius_m':self.radius,
                  'denominator_indices':np.flatnonzero(self.meter.denominator).tolist(),
                  'covered_indices':np.flatnonzero(self.meter.covered).tolist(),'actual_trajectory':self.meter.trajectory,'effective_config':self.settings.values,'config_hash':self.settings.hash,
                  'known_free_m2':float(self.base.free.sum()*self.base.resolution**2)}
            tmp=dest.with_suffix('.json.tmp');tmp.write_text(json.dumps(data,indent=2,allow_nan=False)+'\n');tmp.replace(dest)
        except (ValueError,KeyError,OSError) as e:self.get_logger().error('Report write failed: '+str(e))


def main(args=None):
    rclpy.init(args=args);node=CoverageMeter()
    try:rclpy.spin(node)
    finally:node.destroy_node();rclpy.shutdown()
