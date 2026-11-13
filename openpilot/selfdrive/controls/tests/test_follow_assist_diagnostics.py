"""Exercise planner integration without loading the native MPC solver."""
import ast
from pathlib import Path
from types import SimpleNamespace as NS
import unittest
from unittest.mock import MagicMock

from openpilot.selfdrive.controls.lib.lead_gap_reserve import apply_lead_gap_cap


TREE = ast.parse((Path(__file__).parents[1]/'lib/longitudinal_planner.py').read_text(encoding='utf-8'))
PLANNER = next(n for n in TREE.body if isinstance(n, ast.ClassDef) and n.name == 'LongitudinalPlanner')


class TestFollowAssistanceIntegration(unittest.TestCase):
  def test_diagnostic_rate_and_idle_suppression(self):
    method = next(n for n in PLANNER.body if isinstance(n, ast.FunctionDef) and n.name == 'log_follow_assistance')
    cloudlog = NS(event=MagicMock())
    env = dict(cloudlog=cloudlog)
    exec(compile(ast.Module(body=[method],type_ignores=[]),'actual_planner_logging','exec'),env)
    planner = NS(cutin_anticipation=NS(stage='off',track_id=None,limit=None),
                 lead_gap_reserve=NS(reason='off'),_assist_log_state=None,_assist_log_time=-100.,
                 mpc=NS(desired_distance=10.,t_follow=1.1))
    lead = NS(radarTrackId=34,dRel=15.,vRel=2.)
    emit = lambda t,cap: env['log_follow_assistance'](planner,t,lead,cap,.5,.3)
    for i in range(100):emit(i*.05,None)
    cloudlog.event.assert_not_called()
    planner.lead_gap_reserve.reason = 'recover'
    for i in range(41):emit(10.+i*.05,-.5)
    self.assertEqual(cloudlog.event.call_count,3)
    fields=cloudlog.event.call_args.kwargs
    self.assertEqual(fields['gap_state'],'recover')
    self.assertEqual(fields['lead_id'],34)
    self.assertEqual(fields['before_gap_a'],.5)
    planner.lead_gap_reserve.reason = 'off'
    emit(12.6,None)
    self.assertIsNone(planner._assist_log_state)
    self.assertEqual(cloudlog.event.call_count,4)

  def test_actual_planner_passes_room_and_accel_and_keeps_stronger_braking(self):
    update = next(n for n in PLANNER.body if isinstance(n, ast.FunctionDef) and n.name == 'update')
    index = next(i for i,n in enumerate(update.body) if isinstance(n, ast.Assign) and
                 any(isinstance(t,ast.Name) and t.id == 'gap_cap' for t in n.targets))
    statements=update.body[index:index+3]
    for base,cap,expected in ((.7,.2,.2),(-2.,.2,-2.),(-2.,-1.,-2.)):
      mock=MagicMock(return_value=cap)
      planner=NS(CP=NS(openpilotLongitudinalControl=True),mpc=NS(mode='acc',t_follow=1.1,desired_distance=8.),
                 lead_gap_reserve=NS(update=mock),output_should_stop=False,output_a_target=-.1,dt=.05,
                 j_desired_trajectory=[0.])
      cs=NS(vEgo=12.,gasPressed=False,brakePressed=False)
      class SM(dict):
        def all_checks(self,services):return True
      sm=SM(radarState=NS(leadOne=NS()),selfdriveState=NS(enabled=True))
      sm.logMonoTime={'radarState':int(100e9)}
      env=dict(self=planner,sm=sm,cs=cs,now=100.,fresh=True,reset_state=False,
               carrot=NS(stop_distance=3.6,desireState=0.,xState=0),XState=NS(e2eStop=2,e2eStopped=3),
               output_a_target=base,output_v_target_now=12.,action_t=.2,apply_lead_gap_cap=apply_lead_gap_cap)
      exec(compile(ast.Module(body=statements,type_ignores=[]),'actual_planner_gap_integration','exec'),env)
      self.assertEqual(mock.call_args.kwargs['desired_distance'],8.)
      self.assertEqual(mock.call_args.kwargs['planned_accel'],base)
      self.assertEqual(env['output_a_target'],expected)


if __name__ == '__main__':
  unittest.main()
