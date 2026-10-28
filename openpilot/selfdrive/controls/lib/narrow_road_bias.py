"""Bounded right preference for a confirmed narrow rightmost driving corridor.

The opposing side may be on the left: one-way lane count and line colour alone
are not used as evidence. Camera calibration and global Params are unchanged.
"""
import math
import numpy as np


class NarrowRoadBias:
  def __init__(self):
    self.confirm = 0.
    self.missing = 0.
    self.shift = 0.

  def reset(self):
    self.confirm = self.missing = self.shift = 0.
    return 0.

  def update(self, model, cs, base_path, curvature, requested_shift, *, enabled, dt=.05):
    if not enabled or model is None:
      return self.reset()
    values=(cs.vEgo, curvature, requested_shift, dt)
    if (not all(math.isfinite(v) for v in values) or
        not 0. < dt <= .2 or not 2. <= cs.vEgo <= 60./3.6 or
        cs.leftBlinker or cs.rightBlinker or getattr(cs,'steeringPressed',False) or
        len(model.laneLines)<4 or len(model.laneLineProbs)<4 or len(model.laneLineStds)<4 or
        getattr(model.meta,'laneChangeState',0) != 0 or getattr(model.meta,'desire',0) != 0):
      return self.reset()
    left,right=model.laneLines[1:3]
    x=np.asarray(right.x,dtype=float);rx=np.asarray(right.y,dtype=float)
    lx=np.asarray(left.x,dtype=float);ly=np.asarray(left.y,dtype=float)
    scalars=(model.meta.laneWidthRight,model.laneLineProbs[1],model.laneLineProbs[2],
             model.laneLineProbs[3],model.laneLineStds[1],model.laneLineStds[2])
    if (not all(math.isfinite(v) for v in scalars) or len(x)<3 or len(x)!=len(rx) or len(lx)!=len(ly) or len(lx)<3 or
        not all(np.all(np.isfinite(a)) for a in (x,rx,lx,ly,base_path)) or
        not np.all(np.diff(x)>0.) or not np.all(np.diff(lx)>0.)):
      return self.reset()
    near=(x>=0.) & (x<=15.)
    width=float(np.median(rx[near]-np.interp(x[near],lx,ly))) if np.any(near) else 0.
    right_space=model.meta.laneWidthRight
    outer_right=model.laneLineProbs[3]
    context=(2.6<width<3.3 and 0.<=right_space<2.2 and 0.<=outer_right<.15 and
             model.laneLineProbs[1]>.2 and model.laneLineProbs[2]>.2)
    # Confirm context over time: a brief probability dip on zigzag markings
    # must not erase an otherwise stable corridor. Explicit neighboring-lane
    # evidence, a wide lane, or a full second without support clears it.
    if width>=3.5 or (right_space>=2.6 and outer_right>=.15) or outer_right>=.3:
      return self.reset()
    self.missing=0. if context else self.missing+dt
    self.confirm=max(0.,min(2.,self.confirm+(dt if context else -.5*dt)))
    if self.missing>1.:
      return self.reset()
    reliable=(model.laneLineProbs[1]>.3 and model.laneLineProbs[2]>.4 and
              0.<=model.laneLineStds[1]<.45 and 0.<=model.laneLineStds[2]<.45)
    target=0.;room=None
    future=(base_path[:,0]>=0.) & (base_path[:,0]<=min(15.,cs.vEgo+3.))
    if reliable and np.count_nonzero(future)>=3:
      px=base_path[future,0]
      right_bound=np.interp(px,x,rx)
      # Keep extra room for lane uncertainty; a trusted road edge can only reduce it.
      margin=1.25+.3*model.laneLineStds[2]
      if (len(model.roadEdges)>=2 and len(model.roadEdgeStds)>=2 and
          math.isfinite(model.roadEdgeStds[1]) and 0.<=model.roadEdgeStds[1]<.6):
        edge=model.roadEdges[1];ex=np.asarray(edge.x);ey=np.asarray(edge.y)
        if len(ex)==len(ey) and len(ex)>2 and np.all(np.isfinite(ex)) and np.all(np.isfinite(ey)) and np.all(np.diff(ex)>0.):
          right_bound=np.minimum(right_bound,np.interp(px,ex,ey)-.4*model.roadEdgeStds[1])
      room=max(0.,float(np.min(right_bound-margin-base_path[future,1])))
      if self.confirm>=1.:
        curve_factor=float(np.interp(abs(curvature),[.0015,.005],[1.,0.]))
        width_factor=float(np.interp(width,[2.6,2.75,3.1,3.3],[0.,1.,1.,0.]))
        target=min(.06,max(0.,requested_shift))*curve_factor*width_factor
        target=min(target,room)
    rate=.015 if target>self.shift else .03
    self.shift+=float(np.clip(target-self.shift,-rate*dt,rate*dt))
    if room is not None:self.shift=min(self.shift,room)
    return self.shift
