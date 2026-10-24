from pathlib import Path
import importlib.util,sys,unittest
from types import SimpleNamespace
from unittest.mock import MagicMock,patch
ROOT=Path(__file__).resolve().parents[1]
layout_spec=importlib.util.spec_from_file_location('layout_panel_test',ROOT/'mici/onroad/hud_layout.py')
layout=importlib.util.module_from_spec(layout_spec);sys.modules[layout_spec.name]=layout;layout_spec.loader.exec_module(layout)
spec=importlib.util.spec_from_file_location('lead_brake_state_test',ROOT/'mici/onroad/lead_brake_state.py');state_module=importlib.util.module_from_spec(spec);sys.modules[spec.name]=state_module;spec.loader.exec_module(state_module)


class StateTest(unittest.TestCase):
  def lead(self,d=32.3,v=-1.,present=True):return SimpleNamespace(dRel=d,vRel=v,present=present)
  def test_lead_absent_or_stale_is_hidden(self):
    self.assertIsNone(state_module.lead_brake_state(self.lead(present=False),True))
    self.assertIsNone(state_module.lead_brake_state(self.lead(),False))
  def test_bad_measurements_are_hidden(self):
    for d,v in ((-1,0),(float('nan'),0),(10,float('inf'))):self.assertIsNone(state_module.lead_brake_state(self.lead(d,v),True))
  def test_unknown_when_no_visual_observation(self):
    s=state_module.lead_brake_state(self.lead(),True)
    self.assertEqual(s.state,'UNKNOWN');self.assertEqual(s.status_text,'WAIT')
  def test_candidate_labels(self):
    for state,label in (('ON','ON?'),('OFF','OFF?'),('BAD','WAIT')):
      s=state_module.lead_brake_state(self.lead(),True,SimpleNamespace(state=state))
      self.assertEqual(s.status_text,label)
  def test_relative_speed_directions_and_deadband(self):
    for v,motion,text in ((-1,-1,'CLOSING'),(1,1,'AWAY'),(-.3,0,'STEADY'),(.3,0,'STEADY')):
      s=state_module.lead_brake_state(self.lead(v=v),True)
      self.assertEqual((s.motion,s.motion_text),(motion,text))


class PanelTest(unittest.TestCase):
  @classmethod
  def setUpClass(cls):
    ray=MagicMock();ray.Rectangle=lambda x,y,w,h:SimpleNamespace(x=x,y=y,width=w,height=h);ray.Vector2=lambda x,y:SimpleNamespace(x=x,y=y)
    dependencies={'pyray':ray,'openpilot.selfdrive.ui.mici.onroad':SimpleNamespace(SIDE_PANEL_WIDTH=80),
      'openpilot.selfdrive.ui.mici.onroad.lead_brake_state':state_module,
      'openpilot.selfdrive.ui.mici.onroad.hud_layout':layout,
      'openpilot.system.ui.lib.application':SimpleNamespace(gui_app=SimpleNamespace(font=lambda *a:None),FontWeight=SimpleNamespace(MEDIUM=1,BOLD=2),FONT_SCALE=1.16,font_fallback=lambda f:f),
      'openpilot.system.ui.lib.text_measure':SimpleNamespace(measure_text_cached=lambda font,text,size:SimpleNamespace(x=len(text)*size*.65,y=size))}
    spec=importlib.util.spec_from_file_location('lead_brake_panel_test',ROOT/'mici/onroad/lead_brake_panel.py');cls.module=importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules,dependencies):spec.loader.exec_module(cls.module)

  def test_text_fits_and_is_ascii(self):
    rect=SimpleNamespace(x=20,y=10,width=60,height=220)
    for state in ('ON','OFF','UNKNOWN'):
      ray=self.module.rl;ray.draw_text_ex.reset_mock()
      self.module.LeadBrakePanel.draw(rect,state_module.LeadBrakeState(state,32.3,-1.,-1))
      for call in ray.draw_text_ex.call_args_list:
        _,text,pos,size,*_=call.args
        self.assertTrue(text.isascii());self.assertGreaterEqual(pos.x,rect.x)
        self.assertLessEqual(pos.x+len(text)*size*.65,rect.x+rect.width)
        self.assertGreaterEqual(pos.y,rect.y);self.assertLessEqual(pos.y+size,rect.y+rect.height)

  def test_render_hides_stale_or_absent_lead(self):
    sm=type('SM',(dict,),{})();sm['radarState']=SimpleNamespace(leadOne=SimpleNamespace(present=True,dRel=32.3,vRel=-1.))
    sm.valid={'radarState':True};sm.alive={'radarState':True};sm.recv_time={'radarState':99.}
    detector=SimpleNamespace(enabled=True,latest_observation=lambda now:None)
    panel=self.module.LeadBrakePanel()
    with patch.object(panel,'draw') as draw,patch.object(self.module.time,'monotonic',return_value=100.):
      panel.render(SimpleNamespace(x=0,y=0,width=540,height=240),sm,detector);draw.assert_not_called()
      sm.recv_time['radarState']=100.;panel.render(SimpleNamespace(x=0,y=0,width=540,height=240),sm,detector)
      self.assertEqual(draw.call_count,1)
      sm['radarState'].leadOne.present=False;panel.render(SimpleNamespace(x=0,y=0,width=540,height=240),sm,detector)
      self.assertEqual(draw.call_count,1)

  def test_panel_uses_only_existing_right_strip(self):
    sm=type('SM',(dict,),{})();sm['radarState']=SimpleNamespace(leadOne=SimpleNamespace(present=True,dRel=32.3,vRel=1.))
    sm.valid={'radarState':True};sm.alive={'radarState':True};sm.recv_time={'radarState':100.}
    detector=SimpleNamespace(enabled=True,latest_observation=lambda now:SimpleNamespace(state='ON'))
    panel=self.module.LeadBrakePanel()
    with patch.object(panel,'draw') as draw,patch.object(self.module.time,'monotonic',return_value=100.):
      panel.render(SimpleNamespace(x=15,y=25,width=540,height=240),sm,detector)
      r,s=draw.call_args.args
      self.assertEqual((r.x,r.y,r.width,r.height),(475,95,80,170));self.assertEqual(s.status_text,'ON?')

  def test_three_digit_speed_fits_and_visible_top_is_aligned(self):
    m=self.module
    fake_font=SimpleNamespace(baseSize=200)
    m.gui_app.font=lambda *args:fake_font
    m.rl.get_glyph_info=lambda *args:SimpleNamespace(offsetY=40,image=SimpleNamespace(height=145))
    rect=SimpleNamespace(x=20,y=10,width=60,height=240)
    for speed in (0,37,100,120,199):
      m.rl.draw_text_ex.reset_mock()
      m.LeadBrakePanel.draw_speed(rect,speed,True,(255,255,255,255))
      call=m.rl.draw_text_ex.call_args_list[0]
      _,text,pos,size,*_=call.args
      self.assertGreaterEqual(pos.x,rect.x+3-.001)
      self.assertLessEqual(pos.x+len(text)*size*.65,rect.x+rect.width-3+.001)
      self.assertAlmostEqual(pos.y+40*size*1.16/200,rect.y+layout.HEADER_TOP)
      self.assertLessEqual(145*size*1.16/200,48.001)
    m.gui_app.font=lambda *args:None

  def test_disabled_is_hidden(self):
    panel=self.module.LeadBrakePanel()
    with patch.object(panel,'draw') as draw:
      panel.render(None,None,SimpleNamespace(enabled=False));draw.assert_not_called()


if __name__=='__main__':unittest.main()
