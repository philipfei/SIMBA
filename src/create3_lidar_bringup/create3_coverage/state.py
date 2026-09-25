"""Deterministic mission bookkeeping: clocks and actions are supplied by adapters."""
from dataclasses import dataclass, field
import math
import uuid
from .settings import load_settings


@dataclass
class Mission:
    recovery: dict = field(default_factory=lambda:load_settings()['recovery'])
    generation: str = ''
    token: int = 0
    state: str = 'IDLE'
    started: float = 0.
    deadline: float = 1800.
    reason: str = ''
    root_cause: str = ''
    blacklist: set = field(default_factory=set)
    blocked: list = field(default_factory=list)
    failures: dict = field(default_factory=dict)
    events: list = field(default_factory=list)
    rounds: int = 0
    coverage_finished: bool = False

    def start(self, now, timeout=1800.):
        if self.state not in ('IDLE','CANCELED','FINISHED','FAILED'):
            raise ValueError('Cancel or finish the current task before starting another')
        self.generation=uuid.uuid4().hex;self.token+=1;self.state='PREPARING'
        self.started=now;self.deadline=timeout;self.reason='';self.root_cause=''
        self.blacklist.clear();self.blocked.clear();self.failures.clear();self.events.clear()
        self.rounds=0;self.coverage_finished=False

    def action_id(self):
        self.token+=1
        return (self.generation,self.token)

    def current(self, identity):
        return identity == (self.generation,self.token)

    def change(self, state, reason='', now=0.):
        self.token+=1;self.state=state;self.reason=reason
        if reason:
            if not self.root_cause:self.root_cause=reason
            self.events.append({'time':now,'state':state,'reason':reason})

    def pause(self, reason, now):
        self.change('PAUSED',reason,now)

    def resume(self):
        if self.state!='PAUSED':raise ValueError('Task is not paused')
        self.change('PREPARING')  # Preserve generation, start time, exclusions and retries.

    def expired(self, now):
        return bool(self.generation and not self.coverage_finished and
                    self.state not in ('IDLE','CANCELED','FINISHED','FAILED') and
                    now-self.started>=self.deadline)

    def fail_target(self, target, now):
        n=self.failures.get(target.key,0)+1;self.failures[target.key]=n
        if n>=self.recovery['target_attempts']:
            self.blacklist.add(target.key)
            self.blocked.append({'xy':list(target.end),'radius':self.recovery['blockage_radius_m'],'reason':'TARGET_FAILED','time':now})
        return n

    def excluded(self,target):
        return target.key in self.blacklist or any(
            math.dist(target.start,b['xy'])<=b['radius'] or math.dist(target.end,b['xy'])<=b['radius']
            for b in self.blocked)
