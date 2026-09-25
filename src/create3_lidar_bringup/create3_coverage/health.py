"""Pure continuously evaluated localization trust and command ownership policies."""
import math
import numpy as np
from .settings import load_settings
from .geometry import wrap


class Trust:
    def __init__(self, settings=None):
        self.cfg=(settings or load_settings())['localization']
        self.since=None;self.previous=None;self.reason='LOCALIZATION_UNINITIALIZED'

    def check(self, now, pose, odom, covariance, endpoints, matched, amcl_age,
              sensors_ok=True, tf_ok=True):
        reason=''
        if not sensors_ok:reason='SENSOR_STALE'
        elif not tf_ok or pose is None or odom is None:reason='TF_UNAVAILABLE'
        elif not np.isfinite([*pose,*odom]).all():reason='TF_INVALID'
        elif covariance is None or not np.isfinite(covariance).all():reason='AMCL_INVALID'
        elif amcl_age<-.1 or amcl_age>self.cfg['amcl_max_age_s']:reason='AMCL_STALE'
        else:
            cov=np.asarray(covariance).reshape(6,6)
            sigma=math.sqrt(max(0,float(np.linalg.eigvalsh(cov[:2,:2]).max())))
            yaw_sigma=math.sqrt(max(0,float(cov[5,5])))
            if np.any(np.diag(cov)<0):reason='AMCL_INVALID'
            elif sigma>self.cfg['position_sigma_m']+1e-10:reason='POSITION_UNCERTAIN'
            elif yaw_sigma>math.radians(self.cfg['heading_sigma_deg'])+1e-10:reason='HEADING_UNCERTAIN'
            elif endpoints<self.cfg['min_endpoints']:reason='TOO_FEW_SCAN_ENDPOINTS'
            elif matched/endpoints<self.cfg['matched_ratio']:reason='SCAN_MAP_MISMATCH'
        if not reason and self.previous:
            p,o,t=self.previous
            if now<=t:reason='TIME_REVERSED'
            else:
                # Compare changes in map->odom, not unaligned map and odom displacements.
                def transform(p,o):
                    a=wrap(p[2]-o[2]);c,s=math.cos(a),math.sin(a)
                    return (p[0]-c*o[0]+s*o[1],p[1]-s*o[0]-c*o[1],a)
                a,b=transform(p,o),transform(pose,odom)
                # Rotation around the robot must not magnify a correction by distant odom origin.
                predicted_angle=wrap(p[2]+wrap(odom[2]-o[2]))
                c,s=math.cos(a[2]),math.sin(a[2]);dx,dy=odom[0]-o[0],odom[1]-o[1]
                predicted=(p[0]+c*dx-s*dy,p[1]+s*dx+c*dy)
                if math.dist(predicted,pose[:2])>self.cfg['jump_translation_m'] or abs(wrap(pose[2]-predicted_angle))>math.radians(self.cfg['jump_rotation_deg']):
                    reason='LOCALIZATION_JUMP'
        if reason:self.since=None;self.previous=None
        else:
            if self.since is None:self.since=now
            self.previous=(list(pose),list(odom),now)
        self.reason=reason or ('LOCALIZATION_SETTLING' if now-self.since<self.cfg['settle_s'] else '')
        return not self.reason


class Ownership:
    """One grant epoch; no cached command survives a source hand-off."""
    def __init__(self, linear=.12, angular=.40, acceleration=.15, angular_acceleration=.60, settings=None):
        self.cfg=settings or load_settings()
        self.linear=linear;self.angular=angular;self.acceleration=acceleration;self.angular_acceleration=angular_acceleration
        self.owner='NONE';self.epoch='';self.command=None;self.command_time=-math.inf
        self.lease_time=-math.inf;self.output=(0.,0.);self.stopping=False
        self.stop_at=0.;self.fault='';self.events=[]

    def lease(self, owner, epoch, now):
        if owner not in ('NONE','MANUAL','NAV','NATIVE'):return
        if (owner,epoch)!=(self.owner,self.epoch):
            if self.owner in ('NAV','MANUAL'):self.stopping=True;self.stop_at=now
            self.owner=owner;self.epoch=epoch;self.command=None;self.command_time=-math.inf
        self.lease_time=now

    def receive(self, source, command, now):
        if source==self.owner and all(math.isfinite(x) for x in command):
            self.command=command;self.command_time=now

    def latch(self, reason):
        if not self.fault:self.fault=reason
        if reason and (not self.events or self.events[-1]!=reason):self.events.append(reason)

    def tick(self, now, healthy, stopped, bump=False, dt=.05):
        # Never fight native bump reflexes or an explicitly granted native action.
        if self.owner=='NATIVE' or bump:
            self.output=(0.,0.);self.stopping=False;return None
        valid=healthy and not self.fault and now-self.lease_time<=self.cfg['health']['lease_age_s']
        valid &= self.owner in ('NAV','MANUAL') and self.command is not None and now-self.command_time<=self.cfg['health']['command_age_s']
        if valid:
            v,w=self.command
            if self.owner=='NAV':v=min(self.linear,max(0.,v));w=min(self.angular,max(-self.angular,w))
            else:v=min(self.cfg['motion']['manual_forward_m_s'],max(-self.cfg['motion']['manual_reverse_m_s'],v));w=min(self.cfg['motion']['manual_angular_rad_s'],max(-self.cfg['motion']['manual_angular_rad_s'],w))
            # Idle keyboard zeros do not create a permanent zero stream.
            if abs(v)+abs(w)<1e-9:
                valid=False
            else:
                ov,ow=self.output
                v=min(ov+self.acceleration*dt,max(ov-self.acceleration*dt,v));w=min(ow+self.angular_acceleration*dt,max(ow-self.angular_acceleration*dt,w))
                self.output=(v,w);self.stopping=True;self.stop_at=now
                return self.output
        if self.stopping:
            self.output=(0.,0.)
            if stopped and now-self.stop_at>=self.cfg['health']['stop_burst_s']:self.stopping=False
            return (0.,0.)
        return None
