"""Observers consume confirmed decisions/feedback; never infer success from time alone."""
import logging
import time
from .player import Player

LOG=logging.getLogger(__name__)
REASONS={
 'target_lost_or_changed':'target_lost','multiple_matching_targets':'target_ambiguous',
 'vision_lost':'camera_error','vision_not_ready':'camera_error',
 'vision_timestamp_invalid':'camera_error','camera_restarted':'camera_error',
 'telemetry_stale':'serial_error','target_range_unavailable':'sensor_error',
 'target_approach_blocked':'path_blocked','scan_blocked':'path_blocked',
 'obstacle_stop_replan':'path_blocked','obstacle_extent_requires_intervention':'path_blocked',
 'search_exhausted_or_unreachable':'search_failed','user_stop':'stopped',
 'mission_timeout':'search_failed','navigation_no_progress':'path_blocked',
 'replan_limit':'path_blocked','pose_outside_traversable_map':'execution_error'}

class Session:
    def __init__(self,mode='off',device=None,player=None):
        self.player=player;self.previous=None;self.last_issue=None;self.error_prompt=None
        if player is None:
            try:self.player=Player(mode,device)
            except Exception as exc:LOG.warning('Speech unavailable: %s',exc)

    def emit(self,prompt,fault=False):
        if not self.player:return False
        try:return self.player.submit(prompt,fault=fault)
        except Exception as exc:LOG.warning('Speech event failed: %s',exc);return False

    def cancel(self):
        if self.player:self.player.cancel()

    def guard(self,prompt,fn,*args,**kwargs):
        try:return fn(*args,**kwargs)
        except Exception:self.error_prompt=prompt;raise

    def task6(self,snapshot,telemetry=None):
        state=snapshot['state'];reason=snapshot['reason'];prev=self.previous
        self.previous=dict(snapshot)
        if state=='COMPLETE':
            if (prev and prev['state']=='APPROACH' and reason=='target_reached_and_stopped'
                    and telemetry and max(abs(telemetry['left_rpm']),abs(telemetry['right_rpm']))<=3):
                self.emit('arrived');self.emit('task_complete')
            return
        if state=='SCAN' and prev and prev['state']=='SEARCH' and snapshot.get('station') is not None:
            self.emit('search_point_arrived')
        issue=REASONS.get(reason)
        if reason=='FPGA_sensor_or_motor_fault':
            issue='motor_error' if telemetry and telemetry['flags']&16 else 'sensor_error'
        if state in ('FAILED','STOPPED'):issue=issue or 'execution_error'
        # Repeated state samples do not retrigger a fault. Recovery permits a new episode.
        if issue and issue!=self.last_issue:self.emit(issue,fault=True)
        self.last_issue=issue

    def failed(self,prompt=None):self.emit(prompt or self.error_prompt or 'execution_error',fault=True)
    def close(self):
        if self.player:
            try:self.player.close()
            except Exception as exc:LOG.warning('Speech shutdown failed: %s',exc)

class RouteObserver:
    """Task4 0x28: high waypoint nibble is CURRENT goal, not last reached goal.

    Increment n->n+1 acknowledges P(n+1). Goal 6 is the endpoint, not P7.
    Start/mid-route attachment, telemetry gaps and resets never synthesize arrivals.
    """
    def __init__(self,session):
        self.session=session;self.previous=None;self.route=False;self.seen=set();self.issue=None

    def reset(self):
        self.previous=None;self.route=False;self.seen.clear();self.issue=None;self.session.cancel()

    def lost(self):
        if self.previous is not None:
            self.reset();self.session.failed('serial_error')

    def observe(self,row,now=None):
        now=time.monotonic() if now is None else now
        p=self.previous;state=row['state'];goal=row['waypoint']>>4
        if p and not 0<=now-p['at']<.5:self.reset();p=None
        seq=row['sequence']
        if p:
            delta=(seq-p['sequence'])&65535
            if delta==0 or delta>=32768:return
        self.previous=dict(row,at=now)
        issue=None
        if row['flags']&64:issue='motor_error'
        elif not row['flags']&1:issue='sensor_error'
        elif state==5:issue='execution_error'
        elif row['flags']&4:issue='path_blocked'
        elif state==0 and row['reason']==1 and p and p['state'] not in (0,6):issue='stopped'
        if issue and issue!=self.issue:self.session.failed(issue)
        self.issue=issue
        if state in (0,1,2,5):
            self.route=False;self.seen.clear();return
        if state in (8,9):
            if self.route and p and not issue and goal==(p['waypoint']>>4)+1 and 1<=goal<=6:
                if goal not in self.seen:self.seen.add(goal);self.session.emit('waypoint_p'+str(goal))
            self.route=True
        elif state==6:
            if (self.route and p and p['state']!=6 and goal==6 and not issue and row['reason']==0
                    and max(abs(row['left_rpm']),abs(row['right_rpm']))<=3):
                self.session.emit('arrived');self.session.emit('task_complete')
            self.route=False
        elif state not in (4,7):self.route=False
