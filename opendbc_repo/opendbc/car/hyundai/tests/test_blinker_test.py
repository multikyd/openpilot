import copy
import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace as NS
from opendbc.car.hyundai import blinker_test as m

def vehicle():
 return NS(canValid=True, gearShifter='drive', standstill=True, vEgo=0., brakePressed=True,
           gasPressed=False, doorOpen=False, seatbeltUnlatched=False, leftBlinker=False,rightBlinker=False)
def control(): return NS(enabled=False,latActive=False,longActive=False)

class SenderTest(unittest.TestCase):
 def setUp(self):
  self.tmp=tempfile.TemporaryDirectory()
  self.path=Path(self.tmp.name)/'request';self.status=Path(self.tmp.name)/'status'
  self.sender=m.BlinkerTestSender(self.path,self.status);self.cs=vehicle();self.cc=control()
  self.now=100.;self.req={'id':'test1','start':100.,'expires':100.65,'deadline':103.,'action':'left'}
  self.write()
  self.assertEqual(self.sender.update(self.now,self.cs,self.cc,True),0)
 def tearDown(self): self.tmp.cleanup()
 def write(self): m.write_json(self.path,self.req)
 def update(self, **kwargs): return self.sender.update(self.now,self.cs,self.cc,True,**kwargs)
 def test_left_right_p_d(self):
  for gear in ('park','drive'):
   self.cs.gearShifter=gear
   self.assertEqual(self.update(),3)
   self.req['action']='right';self.write();self.assertEqual(self.update(),4)
   self.req['action']='left';self.write()
 def test_first_packet_is_neutral(self):
  sender=m.BlinkerTestSender(self.path,self.status);self.assertEqual(sender.update(self.now,self.cs,self.cc,True),0)
  self.assertEqual(sender.update(self.now,self.cs,self.cc,True),3)
 def test_no_request_no_unsolicited_zero(self):
  self.sender=m.BlinkerTestSender(self.path,self.status)
  self.path.unlink();self.assertIsNone(self.update())
 def test_release_ten_neutral_packets(self):
  self.assertEqual(self.update(),3)
  self.req['action']='stop';self.write()
  for _ in range(10): self.assertEqual(self.update(),0)
  self.assertIsNone(self.update())
 def test_disconnect_expires(self):
  self.assertEqual(self.update(),3);self.now=100.651;self.assertEqual(self.update(),0)
 def test_cannot_revive_expired_press(self):
  self.update();self.now=100.651;self.update()
  self.req['expires']=101.2;self.write();self.assertEqual(self.update(),0)
 def test_new_press_after_cancel(self):
  self.update();self.req['action']='stop';self.write();self.update()
  self.req.update(id='test2',action='right');self.write();self.assertEqual(self.update(),0);self.assertEqual(self.update(),4)
 def test_three_second_cutoff(self):
  self.update();self.now=103.;self.req['expires']=103.;self.write();self.assertEqual(self.update(),0)
 def test_all_vehicle_conditions_interrupt(self):
  changes={'canValid':False,'gearShifter':'reverse','standstill':False,'vEgo':.05,
           'brakePressed':False,'gasPressed':True,'doorOpen':True,'seatbeltUnlatched':True}
  for field,value in changes.items():
   self.sender=m.BlinkerTestSender(self.path,self.status);self.cs=vehicle();self.assertEqual(self.update(),0);self.assertEqual(self.update(),3)
   setattr(self.cs,field,value);self.assertEqual(self.update(),0,field)
 def test_controls_interrupt(self):
  for field in ('enabled','latActive','longActive'):
   self.sender=m.BlinkerTestSender(self.path,self.status);self.cc=control();self.assertEqual(self.update(),0);self.assertEqual(self.update(),3)
   setattr(self.cc,field,True);self.assertEqual(self.update(),0)
 def test_unknown_gears_and_nan(self):
  for gear in ('reverse','neutral','unknown'):
   self.cs.gearShifter=gear;self.assertNotEqual(m.conditions(self.cs,self.cc),'ok')
  self.cs.gearShifter='drive';self.cs.vEgo=float('nan');self.assertNotEqual(m.conditions(self.cs,self.cc),'ok')
 def test_no_other_vehicle_or_bus(self):
  self.sender=m.BlinkerTestSender(self.path,self.status)
  self.assertIsNone(self.sender.update(self.now,self.cs,self.cc,False))
 def test_opposite_signal_interrupts(self):
  self.assertEqual(self.update(),3);self.cs.rightBlinker=True;self.assertEqual(self.update(),0)
 def test_stock_parking_blocks(self):
  self.sender=m.BlinkerTestSender(self.path,self.status)
  self.assertIsNone(self.update(stock_spas_active=True))
 def test_invalid_json_and_types(self):
  for raw in ('null','[]','42','"value"','{bad','x'*8192):
   self.path.write_text(raw);self.assertEqual(m.read_json(self.path),{})
 def test_malformed_times(self):
  for field,value in (('start',101.),('expires',105.),('deadline',104.),('expires','nan'),('expires',None)):
   req=copy.deepcopy(self.req);req[field]=value;self.path.write_text(json.dumps(req))
   sender=m.BlinkerTestSender(self.path,self.status)
   self.assertIsNone(sender.update(self.now,self.cs,self.cc,True))
 def test_telemetry_failure_does_not_crash(self):
  self.sender.status_path=Path(self.tmp.name)/'absent'/'status';self.assertEqual(self.update(),3)

if __name__ == '__main__': unittest.main()
