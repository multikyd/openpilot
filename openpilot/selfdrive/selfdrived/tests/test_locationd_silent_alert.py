"""Exercise production alert factories/mapping without platform UI dependencies."""
import ast
from pathlib import Path
from types import SimpleNamespace as NS
import unittest


def load_alerts(mici):
  path=Path(__file__).parents[1]/'events.py'
  tree=ast.parse(path.read_text(encoding='utf8'))
  wanted={'Alert','NoEntryAlert','SoftDisableAlert','ImmediateDisableAlert','soft_disable_alert'}
  nodes=[n for n in tree.body if isinstance(n,(ast.ClassDef,ast.FunctionDef)) and n.name in wanted]
  # Only remove import-dependent annotations; execute original bodies/defaults.
  for node in nodes:
    for child in ast.walk(node):
      if isinstance(child,ast.FunctionDef):child.returns=None
      if isinstance(child,ast.arg):child.annotation=None
  audible=NS(none=0,warningSoft=1,warningImmediate=2,refuse=3)
  env=dict(DT_CTRL=.01,HARDWARE=NS(get_device_type=lambda:'mici' if mici else 'tizi'),
           AlertStatus=NS(normal=0,userPrompt=1,critical=2),AlertSize=NS(mid=1,full=2),
           Priority=NS(LOW=1,MID=2,HIGHEST=3),VisualAlert=NS(none=0,steerRequired=1),
           AudibleAlert=audible)
  exec(compile(ast.Module(body=nodes,type_ignores=[]),str(path),'exec'),env)
  mapping=next(n.value for n in tree.body if isinstance(n,ast.AnnAssign) and
               isinstance(n.target,ast.Name) and n.target.id=='EVENTS')
  entry=next(value for key,value in zip(mapping.keys,mapping.values) if
             isinstance(key,ast.Attribute) and key.attr=='locationdTemporaryError')
  env['ET']=NS(NO_ENTRY='noEntry',SOFT_DISABLE='softDisable')
  alerts=eval(compile(ast.Expression(entry),str(path),'eval'),env)
  return env,alerts


class TestLocationdSilentAlert(unittest.TestCase):
  def presentation(self,alert):
    return tuple(getattr(alert,k) for k in ('alert_text_1','alert_text_2','alert_status',
                 'alert_size','priority','visual_alert','duration','creation_delay'))

  def test_no_entry_is_silent_and_preserves_display_on_both_devices(self):
    for mici in (False,True):
      env,alerts=load_alerts(mici)
      actual=alerts['noEntry']
      reference=env['NoEntryAlert']('locationd Temporary Error')
      self.assertEqual(actual.audible_alert,env['AudibleAlert'].none)
      self.assertEqual(self.presentation(actual),self.presentation(reference))

  def test_soft_disable_and_last_half_second_are_both_silent(self):
    env,alerts=load_alerts(True)
    reference=env['soft_disable_alert']('locationd Temporary Error')
    for remaining in (300,100,50,49,1,0):
      args=(None,None,None,True,remaining,None)
      actual=alerts['softDisable'](*args)
      self.assertEqual(actual.audible_alert,env['AudibleAlert'].none)
      self.assertEqual(self.presentation(actual),self.presentation(reference(*args)))

  def test_unrelated_alerts_keep_original_sounds(self):
    env,_=load_alerts(False)
    self.assertEqual(env['NoEntryAlert']('Other Error').audible_alert,env['AudibleAlert'].refuse)
    default=env['soft_disable_alert']('Other Error')
    self.assertEqual(default(None,None,None,True,300,None).audible_alert,env['AudibleAlert'].warningSoft)
    self.assertEqual(default(None,None,None,True,0,None).audible_alert,env['AudibleAlert'].warningImmediate)


if __name__=='__main__':unittest.main()
