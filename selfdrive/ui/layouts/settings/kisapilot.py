from openpilot.common.params import Params
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.widgets import Widget, DialogResult
from openpilot.system.ui.widgets.list_view import toggle_item, button_item, numeric_item, single_button_item
from openpilot.system.ui.widgets.scroller_tici import Scroller
from openpilot.system.ui.widgets.confirm_dialog import ConfirmDialog, alert_dialog
from openpilot.system.ui.lib.application import gui_app
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.system.ui.widgets.option_dialog import MultiOptionDialog
import os
import pyray as rl

BUTTON_HEIGHT = 90
BUTTON_PADDING = 10
HIGHLIGHT_COLOR = (34, 139, 34, 241)
DEFAULT_COLOR = (128, 128, 128, 255)
TEXT_COLOR = (255, 255, 255, 255)
TEXT_SIZE = 55
CORNER_RADIUS = 5

TOGGLES = [
  {"n": "0", "param": "PutPrebuiltOn", "title": "Use Smart Prebuilt", "description": "Create a Prebuilt file and speed up booting. When this function is turned on, the booting speed is accelerated using the cache, and if you press the update button in the menu after modifying the code, or if you rebooted with the 'gi' command in the command window, remove it automatically and compile it."},
  {"n": "1", "param": "KisaBlindSpotDetect", "title": "Display BSM Status", "description": "If a car is detected in the rear, it will be displayed on the screen."},
  {"n": "2", "param": "KisaEnableLogger", "title": "Enable Driving Log Record", "description": "Record the driving log locally for data analysis. Only loggers are activated and not uploaded to the server."},
]

BUTTONS = [
  {"n": "0", "title": "Delete All Driving Logs", "text": "RUN", "callback": lambda: gui_app.set_modal_overlay(
    ConfirmDialog("Delete all saved driving logs. Do you want to proceed?", "OK"),
    callback=lambda result: os.system("rm -rf /data/media/0/realdata/*") if result == DialogResult.CONFIRM else None),
    "description": "This removes all driving logs under /data/media/0/realdata/."},
]

NUMERICS = [
  {"n": "0", "title": "LaneChange Delay(x0.1s)", "param": "LaneChangeDelay", "min_value": 0, "max_value": 100, "step": 5, "decimals": 0, "value_type": "INT", "special_texts": {"0": "RightNow"}, "description": "Set the delay time after turn signal operation before lane change."},
  {"n": "1", "title": "Debug UI", "param": "ShowDebugUI", "min_value": 0, "max_value": 2, "step": 1, "decimals": 0, "value_type": "INT", "special_texts": {"0": "None", "1": "Dev State", "2": "Dev+Safety"}, "description": "Debug UI"},
  {"n":"2","title":"HYUNDAI: CAMERA SCC","param":"HyundaiCameraSCC","min_value":0,"max_value":3,"step":1,"decimals":0,"value_type":"INT","special_texts":{"0": "None","1":"CAMERA SCC","2":"Cruise State","3":"Stock Long"},"description":"1:Connect the SCC's CAN line to CAM, 2:Sync Cruise state, 3:StockLong"},
  {"n":"3","title":"CANFD: HDA2 mode","param":"CanfdHDA2","min_value":0,"max_value":2,"step":1,"decimals":0,"value_type":"INT","special_texts":{"0":"None","1":"HDA2","2":"HDA2+BSM"},"description":"1:HDA2,2:HDA2+BSM"},
  {"n":"4","title":"Enable Radar Track","param":"EnableRadarTracks","min_value":-1,"max_value":3,"step":1,"decimals":0,"value_type":"INT","special_texts":{"-1":"None","0":"None","1":"RadarTrack","2":"StockRadar"},"description":"1:Enable RadarTrack, -1,2:Disable use HKG SCC radar at all times"},
  {"n":"5","title":"Auto Cruise control","param":"AutoCruiseControl","min_value":0,"max_value":3,"step":1,"decimals":0,"value_type":"INT","description":"Softhold, Auto Cruise ON/OFF control"},
  {"n":"6","title":"CRUISE: Auto ON distance(0cm)","param":"CruiseOnDist","min_value":0,"max_value":2500,"step":50,"decimals":0,"value_type":"INT","description":"When GAS/Brake is OFF, Cruise ON when the lead car gets closer."},
  {"n":"7","title":"Auto Engage control on start","param":"AutoEngage","min_value":0,"max_value":2,"step":1,"decimals":0,"value_type":"INT","special_texts":{"1":"SteerEnable","2":"Steer/Cruise Engage"},"description":"1:SteerEnable, 2:Steer/Cruise Engage"},
  {"n":"8","title":"Auto AccelTok speed","param":"AutoGasTokSpeed","min_value":0,"max_value":200,"step":5,"decimals":0,"value_type":"INT","description":"Gas(Accel)Tok enable speed"},
  {"n":"9","title":"Read Cruise Speed from PCM","param":"SpeedFromPCM","min_value":0,"max_value":3,"step":1,"decimals":0,"value_type":"INT","description":"Toyota must set to 1, Honda 3"},
  {"n":"10","title":"Sound Volume(100%)","param":"SoundVolumeAdjust","min_value":5,"max_value":200,"step":5,"decimals":0,"value_type":"INT"},
  {"n":"11","title":"Sound Volume, Engage(10%)","param":"SoundVolumeAdjustEngage","min_value":5,"max_value":200,"step":5,"decimals":0,"value_type":"INT"},
  {"n":"12","title":"Record Road camera(0)","param":"RecordRoadCam","min_value":0,"max_value":2,"step":1,"decimals":0,"value_type":"INT","special_texts":{"0":"None","1":"RoadCam","2":"Road+Wide"},"description":"1:RoadCam, 2:RoadCam+WideRoadCam"},
  {"n":"13","title":"Use HDP(CCNC)(0)","param":"HDPuse","min_value":0,"max_value":2,"step":1,"decimals":0,"value_type":"INT","special_texts":{"0":"None","1":"with APN","2":"Always"},"description":"1:While Using APN, 2:Always"},
  {"n":"14","title":"NNFF","param":"NNFF","min_value":0,"max_value":1,"step":1,"decimals":0,"value_type":"INT","description":"Twilsonco's NNFF(Reboot required)"},
  {"n":"15","title":"NNFFLite","param":"NNFFLite","min_value":0,"max_value":1,"step":1,"decimals":0,"value_type":"INT","description":"Twilsonco's NNFF-Lite(Reboot required)"},
  {"n":"16","title":"Auto sync Cruise speed","param":"AutoGasSyncSpeed","min_value":0,"max_value":1,"step":1,"decimals":0,"value_type":"INT","description":"If accelerator is pressed and speed exceeds set speed, update set speed"},
  {"n":"17","title":"Enable Software Menu","param":"SoftwareMenu","min_value":0,"max_value":1,"step":1,"decimals":0,"value_type":"INT"},

  {"n":"18","title":"Laneline Mode Speed(0)","param":"UseLaneLineSpeed","min_value":0,"max_value":200,"step":5,"decimals":0,"value_type":"INT","description":"Laneline mode, lat_mpc control used"},
  {"n":"19","title":"Laneline Mode Curve Speed(0)","param":"UseLaneLineCurveSpeed","min_value":0,"max_value":200,"step":5,"decimals":0,"value_type":"INT","description":"Laneline mode, high speed only"},
  {"n":"20","title":"AdjustLaneOffset(0)cm","param":"AdjustLaneOffset","min_value":0,"max_value":500,"step":5,"decimals":0,"value_type":"INT"},
  {"n":"21","title":"LaneChange Need Torque","param":"LaneChangeNeedTorque","min_value":-1,"max_value":1,"step":1,"decimals":0,"value_type":"INT","special_texts":{"-1":"Disable","0":"Nudgeless","1":"Nudge"},"description":"-1:Disable lanechange, 0: no need torque, 1:need torque"},
  {"n":"22","title":"LaneChange Delay(x0.1s)","param":"LaneChangeDelay","min_value":0,"max_value":100,"step":5,"decimals":0,"value_type":"INT","description":"x0.1sec"},
  {"n":"23","title":"LaneChange BSD","param":"LaneChangeBsd","min_value":-1,"max_value":1,"step":1,"decimals":0,"value_type":"INT","special_texts":{"-1":"Ignore BSD","0":"BSD Detect","1":"Block Steer"},"description":"-1:ignore bsd, 0:BSD detect, 1: block steer torque"},
  {"n":"24","title":"LAT:SteerRatiox0.1(0)","param":"CustomSR","min_value":0,"max_value":300,"step":1,"decimals":0,"value_type":"INT","description":"Custom SteerRatio"},
  {"n":"25","title":"LAT:SteerRatioRatex0.01(100)","param":"SteerRatioRate","min_value":30,"max_value":170,"step":1,"decimals":0,"value_type":"INT","description":"SteerRatio apply rate"},
  {"n":"26","title":"LAT:PathOffset","param":"PathOffset","min_value":-150,"max_value":150,"step":1,"decimals":0,"value_type":"INT","description":"(-)left, (+)right"},
  {"n":"27","title":"LAT:SteerActuatorDelay(30)","param":"SteerActuatorDelay","min_value":0,"max_value":100,"step":1,"decimals":0,"value_type":"INT","description":"x0.01, 0:LiveDelay"},
  {"n":"28","title":"LAT:LatSmoothSec(13)","param":"LatSmoothSec","min_value":1,"max_value":30,"step":1,"decimals":0,"value_type":"INT","description":"x0.01"},
  {"n":"29","title":"LAT:TorqueCustom(0)","param":"LateralTorqueCustom","min_value":0,"max_value":2,"step":1,"decimals":0,"value_type":"INT"},
  {"n":"30","title":"LAT:TorqueAccelFactor(2500)","param":"LateralTorqueAccelFactor","min_value":1000,"max_value":6000,"step":10,"decimals":0,"value_type":"INT"},
  {"n":"31","title":"LAT:TorqueFriction(100)","param":"LateralTorqueFriction","min_value":0,"max_value":1000,"step":10,"decimals":0,"value_type":"INT"},
  {"n":"32","title":"LAT:CustomSteerMax(0)","param":"CustomSteerMax","min_value":0,"max_value":30000,"step":5,"decimals":0,"value_type":"INT"},
  {"n":"33","title":"LAT:CustomSteerDeltaUp(0)","param":"CustomSteerDeltaUp","min_value":0,"max_value":50,"step":1,"decimals":0,"value_type":"INT"},
  {"n":"34","title":"LAT:CustomSteerDeltaDown(0)","param":"CustomSteerDeltaDown","min_value":0,"max_value":50,"step":1,"decimals":0,"value_type":"INT"},
  {"n":"35","title":"LONG:P Gain(100)","param":"LongTuningKpV","min_value":0,"max_value":150,"step":5,"decimals":0,"value_type":"INT"},
  {"n":"36","title":"LONG:I Gain(0)","param":"LongTuningKiV","min_value":0,"max_value":2000,"step":5,"decimals":0,"value_type":"INT"},
  {"n":"37","title":"LONG:FF Gain(100)","param":"LongTuningKf","min_value":0,"max_value":200,"step":5,"decimals":0,"value_type":"INT"},
  {"n":"38","title":"LONG:ActuatorDelay(20)","param":"LongActuatorDelay","min_value":0,"max_value":200,"step":5,"decimals":0,"value_type":"INT"},
  {"n":"39","title":"LONG:VEgoStopping(50)","param":"VEgoStopping","min_value":1,"max_value":100,"step":5,"decimals":0,"value_type":"INT","description":"Stopping factor"},
  {"n":"40","title":"LONG:Radar Reaction Factor(100)","param":"RadarReactionFactor","min_value":0,"max_value":200,"step":10,"decimals":0,"value_type":"INT"},
  {"n":"41","title":"LONG:StoppingStartAccelx0.01(-40)","param":"StoppingAccel","min_value":-100,"max_value":0,"step":5,"decimals":0,"value_type":"INT"},
  {"n":"42","title":"LONG:StopDistance (600)cm","param":"StopDistanceCarrot","min_value":300,"max_value":1000,"step":10,"decimals":0,"value_type":"INT"},
  {"n":"43","title":"LONG:Jerk Lead Factor (0)","param":"JLeadFactor3","min_value":0,"max_value":100,"step":5,"decimals":0,"value_type":"INT","description":"x0.01"},
  {"n":"44","title":"ACCEL:0~10km/h(160)","param":"CruiseMaxVals0","min_value":5,"max_value":250,"step":5,"decimals":0,"value_type":"INT","description":"Acceleration needed at specified speed.(x0.01m/s^2)"},
  {"n":"45","title":"ACCEL:10~40km/h(160)","param":"CruiseMaxVals1","min_value":5,"max_value":250,"step":5,"decimals":0,"value_type":"INT","description":"Acceleration needed at specified speed.(x0.01m/s^2)"},
  {"n":"46","title":"ACCEL:40~60km/h(120)","param":"CruiseMaxVals2","min_value":5,"max_value":250,"step":5,"decimals":0,"value_type":"INT","description":"Acceleration needed at specified speed.(x0.01m/s^2)"},
  {"n":"47","title":"ACCEL:60~80km/h(100)","param":"CruiseMaxVals3","min_value":5,"max_value":250,"step":5,"decimals":0,"value_type":"INT","description":"Acceleration needed at specified speed.(x0.01m/s^2)"},
  {"n":"48","title":"ACCEL:80~110km/h(80)","param":"CruiseMaxVals4","min_value":5,"max_value":250,"step":5,"decimals":0,"value_type":"INT","description":"Acceleration needed at specified speed.(x0.01m/s^2)"},
  {"n":"49","title":"ACCEL:110~140km/h(70)","param":"CruiseMaxVals5","min_value":5,"max_value":250,"step":5,"decimals":0,"value_type":"INT","description":"Acceleration needed at specified speed.(x0.01m/s^2)"},
  {"n":"50","title":"ACCEL:140~km/h(60)","param":"CruiseMaxVals6","min_value":5,"max_value":250,"step":5,"decimals":0,"value_type":"INT","description":"Acceleration needed at specified speed.(x0.01m/s^2)"},

  {"n":"51","title":"Brightness Ratio(%)","param":"ShowCustomBrightness","min_value":10,"max_value":100,"step":10,"decimals":0,"value_type":"INT"},

  {"n":"52","title":"Button:Cruise Button Mode","param":"CruiseButtonMode","min_value":0,"max_value":2,"step":1,"decimals":0,"value_type":"INT","special_texts":{"0":"Normal","1":"User1","2":"User2"},"description":"0:Normal,1:User1,2:User2"},
  {"n":"53","title":"Button:Cancel Button Mode","param":"CancelButtonMode","min_value":0,"max_value":1,"step":1,"decimals":0,"value_type":"INT","special_texts":{"0":"Long","1":"Long+Lat"},"description":"0:Long,1:Long+Lat"},
  {"n":"54","title":"Button:LFA Button Mode","param":"LfaButtonMode","min_value":0,"max_value":1,"step":1,"decimals":0,"value_type":"INT","special_texts":{"0":"Normal","1":"Decel&Stop&LeadCarReady"},"description":"0:Normal,1:Decel&Stop&LeadCarReady"},
  {"n":"55","title":"Button:Cruise Speed Unit(Basic)","param":"CruiseSpeedUnitBasic","min_value":1,"max_value":20,"step":1,"decimals":0,"value_type":"INT"},
  {"n":"56","title":"Button:Cruise Speed Unit(Extra)","param":"CruiseSpeedUnit","min_value":1,"max_value":20,"step":1,"decimals":0,"value_type":"INT"},
  {"n":"57","title":"CRUISE:Eco control(4km/h)","param":"CruiseEcoControl","min_value":0,"max_value":10,"step":1,"decimals":0,"value_type":"INT","description":"Temporarily increasing the set speed to improve fuel efficiency."},
  {"n":"58","title":"CRUISE:Auto speed up (0%)","param":"AutoSpeedUptoRoadSpeedLimit","min_value":0,"max_value":200,"step":10,"decimals":0,"value_type":"INT","description":"Auto speed up based on the lead car up to RoadSpeedLimit."},
  {"n":"59","title":"GAP1:Apply TFollow (110)x0.01s","param":"TFollowGap1","min_value":70,"max_value":300,"step":5,"decimals":0,"value_type":"INT"},
  {"n":"60","title":"GAP2:Apply TFollow (120)x0.01s","param":"TFollowGap2","min_value":70,"max_value":300,"step":5,"decimals":0,"value_type":"INT"},
  {"n":"61","title":"GAP3:Apply TFollow (160)x0.01s","param":"TFollowGap3","min_value":70,"max_value":300,"step":5,"decimals":0,"value_type":"INT"},
  {"n":"62","title":"GAP4:Apply TFollow (180)x0.01s","param":"TFollowGap4","min_value":70,"max_value":300,"step":5,"decimals":0,"value_type":"INT"},
  {"n":"63","title":"Dynamic GAP control","param":"DynamicTFollow","min_value":0,"max_value":100,"step":5,"decimals":0,"value_type":"INT"},
  {"n":"64","title":"Dynamic GAP control (LaneChange)","param":"DynamicTFollowLC","min_value":0,"max_value":100,"step":5,"decimals":0,"value_type":"INT"},
  {"n":"65","title":"DRIVEMODE: Select","param":"MyDrivingMode","min_value":1,"max_value":4,"step":1,"decimals":0,"value_type":"INT","special_texts":{"1":"ECO","2":"SAFE","3":"NORMAL","4":"HIGH"},"description":"1:ECO,2:SAFE,3:NORMAL,4:HIGH"},
  {"n":"66","title":"DRIVEMODE: Auto","param":"MyDrivingModeAuto","min_value":0,"max_value":1,"step":1,"decimals":0,"value_type":"INT","description":"NORMAL mode only"},
  {"n":"67","title":"TrafficLight DetectMode","param":"TrafficLightDetectMode","min_value":0,"max_value":2,"step":1,"decimals":0,"value_type":"INT","special_texts":{"0":"None","1":"Stopping only","2":"Stop & Go"},"description":"0:None, 1:Stopping only, 2: Stop & Go"},
  {"n":"68","title":"AChangeCostStarting","param":"AChangeCostStarting","min_value":0,"max_value":200,"step":10,"decimals":0,"value_type":"INT"},
  {"n":"69","title":"TrafficStopDistanceAdjust","param":"TrafficStopDistanceAdjust","min_value":-600,"max_value":600,"step":50,"decimals":0,"value_type":"INT"},

  {"n":"70","title":"CURVE:Lower limit speed(30)","param":"AutoCurveSpeedLowerLimit","min_value":30,"max_value":200,"step":5,"decimals":0,"value_type":"INT","description":"When you approach a curve, reduce your speed. Minimum speed"},
  {"n":"71","title":"CURVE:Auto Control ratio(100%)","param":"AutoCurveSpeedFactor","min_value":50,"max_value":300,"step":1,"decimals":0,"value_type":"INT"},
  {"n":"72","title":"CURVE:Aggressiveness (100%)","param":"AutoCurveSpeedAggressiveness","min_value":50,"max_value":300,"step":1,"decimals":0,"value_type":"INT"},
  {"n":"73","title":"RoadSpeedLimitOffset(-1)","param":"AutoRoadSpeedLimitOffset","min_value":-1,"max_value":100,"step":1,"decimals":0,"value_type":"INT","description":"-1:NotUsed,RoadLimitSpeed+Offset"},
  {"n":"74","title":"Auto Roadlimit Speed adjust (50%)","param":"AutoRoadSpeedAdjust","min_value":-1,"max_value":100,"step":5,"decimals":0,"value_type":"INT"},
  {"n":"75","title":"SpeedCamDecelEnd(6s)","param":"AutoNaviSpeedCtrlEnd","min_value":3,"max_value":20,"step":1,"decimals":0,"value_type":"INT","description":"Sets the deceleration completion point. A larger value completes deceleration farther away from the camera."},
  {"n":"76","title":"NaviSpeedControlMode(2)","param":"AutoNaviSpeedCtrlMode","min_value":0,"max_value":3,"step":1,"decimals":0,"value_type":"INT","special_texts":{"0":"No slowdown","1":"Speed Cam","2":"SpeedCam+Bump","3":"Cam+Bump+Mobile"},"description":"0:No slowdown, 1: speed camera, 2: + accident prevention bump, 3: + mobile camera"},
  {"n":"77","title":"SpeedCamDecelRatex0.01m/s^2(80)","param":"AutoNaviSpeedDecelRate","min_value":10,"max_value":200,"step":10,"decimals":0,"value_type":"INT","description":"Lower number, slows down from a greater distance"},
  {"n":"78","title":"SpeedCamSafetyFactor(105%)","param":"AutoNaviSpeedSafetyFactor","min_value":80,"max_value":120,"step":1,"decimals":0,"value_type":"INT"},
  {"n":"79","title":"SpeedBumpTimeDistance(1s)","param":"AutoNaviSpeedBumpTime","min_value":1,"max_value":50,"step":1,"decimals":0,"value_type":"INT"},
  {"n":"80","title":"SpeedBumpSpeed(35Km/h)","param":"AutoNaviSpeedBumpSpeed","min_value":10,"max_value":100,"step":5,"decimals":0,"value_type":"INT"},
  {"n":"81","title":"NaviCountDown mode(2)","param":"AutoNaviCountDownMode","min_value":0,"max_value":2,"step":1,"decimals":0,"value_type":"INT","special_texts":{"0":"off","1":"tbt+camera","2":"tbt+cam+bump"},"description":"0: off, 1:tbt+camera, 2:tbt+camera+bump"},
  {"n":"82","title":"Turn Speed control mode(1)","param":"TurnSpeedControlMode","min_value":0,"max_value":3,"step":1,"decimals":0,"value_type":"INT","special_texts":{"0":"off","1":"vision","2":"vision+route","3":"route"},"description":"0: off, 1:vision, 2:vision+route, 3: route"},
  {"n":"83","title":"Map TurnSpeed Factor(100)","param":"MapTurnSpeedFactor","min_value":50,"max_value":300,"step":5,"decimals":0,"value_type":"INT"},
  {"n":"84","title":"Model TurnSpeed Factor(0)","param":"ModelTurnSpeedFactor","min_value":0,"max_value":80,"step":10,"decimals":0,"value_type":"INT"},
  {"n":"85","title":"ATC:Auto turn control(0)","param":"AutoTurnControl","min_value":0,"max_value":3,"step":1,"decimals":0,"value_type":"INT","special_texts":{"0":"None","1":"LC","2":"LC + SPD","3":"SPD"},"description":"0:None, 1: lane change, 2: lane change + speed, 3: speed"},
  {"n":"86","title":"ATC:Turn Speed (20)","param":"AutoTurnControlSpeedTurn","min_value":0,"max_value":100,"step":5,"decimals":0,"value_type":"INT","description":"0:None, turn speed"},
  {"n":"87","title":"ATC:Turn CtrlDistTime (6)","param":"AutoTurnControlTurnEnd","min_value":0,"max_value":30,"step":1,"decimals":0,"value_type":"INT","description":"dist=speed*time"},
  {"n":"88","title":"ATC Auto Map Change(0)","param":"AutoTurnMapChange","min_value":0,"max_value":1,"step":1,"decimals":0,"value_type":"INT"},
  {"n":"89","title":"RadarTrack Cutin Factor(0)","param":"RadarLatFactor","min_value":0,"max_value":1000,"step":50,"decimals":0,"value_type":"INT","description":"Higher values increase sensitivity to cutting-in or exiting vehicles"},
  {"n":"90","title":"LONG:AChangeCost2(30)","param":"AChangeCost2","min_value":10,"max_value":50,"step":1,"decimals":0,"value_type":"INT","description":"Higher values more smooth stopping"}
]


class KisaPilotLayout(Widget):
  def __init__(self):
    super().__init__()
    self._params = Params()

    self._select_car_dialog = None
    self._select_car_btn = single_button_item(
      lambda: self._params.get("CarSelected3") or "Select Your Car",
      callback=self._show_car_selection_dialog
    )

    self._toggles, self._buttons, self._numeric = [], [], []
    self._param_mapping = []
    self._meta_mapping = {}

    self.can_type = str(self._params.get("KisaCANType", return_default=True)).strip().upper()
    self.scc_type = str(self._params.get("KisaSCCType", return_default=True)).strip().upper()

    # Toggle widgets
    for meta in TOGGLES:
      key = meta["param"]
      initial = self._params.get_bool(key)
      w = toggle_item(meta["title"], description=meta.get("description", ""), initial_state=initial,
        callback=lambda state, k=key: self._params.put_bool(k, state))
      self._toggles.append(w)
      self._param_mapping.append((key, w))
      self._meta_mapping[w] = meta

    # Button widgets
    for meta in BUTTONS:
      btn = button_item(meta["title"], meta["text"], description=meta.get("description", ""), callback=meta["callback"])
      self._buttons.append(btn)

    # Numeric widgets
    for meta in NUMERICS:
      key = meta["param"]
      val_type = meta.get("value_type", "INT")
      step = meta.get("step", 1)
      min_v = meta.get("min_value")
      max_v = meta.get("max_value")
      decimals = meta.get("decimals", 0)
      w = numeric_item(meta["title"], param_key=key, description=meta.get("description", ""), value_type=val_type, min_value=min_v, max_value=max_v, step=step, decimals=decimals, special_texts=meta.get("special_texts"))
      self._numeric.append(w)
      self._param_mapping.append((key, w))

    self._menu_titles = ["Str/UI", "Lat/Lon", "B/Ga/St", "C/Sf/Nv", "Advance"]
    self._menu_items = [
      # Start/UI
      [
        self._select_car_btn,
        self._numeric[2],
        self._numeric[3],
        self._numeric[4],
        self._numeric[5],
        self._numeric[6],
        self._numeric[7],
        self._numeric[8],
        self._numeric[9],
        self._numeric[10],
        self._numeric[11],
        self._numeric[51],
        self._numeric[12],
        self._numeric[13],
        self._numeric[16],
        self._toggles[1],
      ],

      # LAT/LONG
      [
        self._numeric[20],
        self._numeric[24],
        self._numeric[25],
        self._numeric[26],
        self._numeric[27],
        self._numeric[28],
        self._numeric[29],
        self._numeric[30],
        self._numeric[31],
        self._numeric[32],
        self._numeric[33],
        self._numeric[34],
        self._numeric[89],
        self._numeric[35],
        self._numeric[36],
        self._numeric[38],
        self._numeric[39],
        self._numeric[40],
        self._numeric[41],
        self._numeric[42],
        self._numeric[43],
        self._numeric[90],
        self._numeric[44],
        self._numeric[45],
        self._numeric[46],
        self._numeric[47],
        self._numeric[48],
        self._numeric[49],
        self._numeric[50],
      ],

      # Button/Gap/St
      [
        self._numeric[52],
        self._numeric[53],
        self._numeric[54],
        self._numeric[55],
        self._numeric[56],
        self._numeric[57],
        self._numeric[58],
        self._numeric[59],
        self._numeric[60],
        self._numeric[61],
        self._numeric[62],
        self._numeric[63],
        self._numeric[64],
        self._numeric[65],
        self._numeric[66],
        self._numeric[67],
        self._numeric[68],
        self._numeric[69],
      ],

      # CV/Safe/Nav
      [
        self._numeric[18],
        self._numeric[19],
        self._numeric[21],
        self._numeric[22],
        self._numeric[23],
        self._numeric[70],
        self._numeric[71],
        self._numeric[72],
        self._numeric[73],
        self._numeric[74],
        self._numeric[75],
        self._numeric[76],
        self._numeric[77],
        self._numeric[78],
        self._numeric[79],
        self._numeric[80],
        self._numeric[81],
        self._numeric[82],
        self._numeric[83],
        self._numeric[84],
        self._numeric[85],
        self._numeric[86],
        self._numeric[87],
        self._numeric[88],
      ],

      # Advance
      [
        self._toggles[0],  # PutPrebuiltOn : bool
        #self._toggles[1],  # UFCModeEnabled : bool
        #self._toggles[2],  # LFAButtonEngagement : bool
        self._toggles[2],  # KisaEnableLogger : bool
        self._buttons[0],  # Delete All Driving Logs
        #self._toggles[8],  # AutoEnable : bool
        #self._numeric[2],  # AutoEnableSpeed : int
        #self._numeric[10], # UseLegacyLaneModel : int
        self._numeric[1],  # ShowDebugUI : int
      ],
    ]
    self._current_menu = 0

    self._scroller = Scroller(self._menu_items[self._current_menu], line_separator=True, spacing=0)
    self._press_offset_filter = FirstOrderFilter(0.0, 0.3, 1 / gui_app.target_fps)

    ui_state.add_offroad_transition_callback(self._update_items)

  def _set_menu(self, menu_index):
    self._current_menu = menu_index
    self._scroller._items = self._menu_items[menu_index]
    for item in self._scroller._items:
      item.set_touch_valid_callback(self._scroller.scroll_panel.is_touch_valid)
      item.show_event()
    self._scroller.scroll_panel.set_offset(0.0)
    self._update_items()

  def _handle_mouse_release(self, mouse_pos):
    total_width = self._rect.width
    button_width = (total_width - BUTTON_PADDING * (len(self._menu_titles) + 1)) / len(self._menu_titles)
    x = self._rect.x + BUTTON_PADDING
    y = self._rect.y + BUTTON_PADDING

    for idx, title in enumerate(self._menu_titles):
      btn_rect = rl.Rectangle(int(x), int(y), int(button_width), BUTTON_HEIGHT)
      if rl.check_collision_point_rec(mouse_pos, btn_rect):
        self._set_menu(idx)
        return True
      x += button_width + BUTTON_PADDING
    return False

  def _render(self, rect):
    self._rect = rect

    total_width = rect.width
    button_width = (total_width - BUTTON_PADDING * (len(self._menu_titles) + 1)) / len(self._menu_titles)
    x = rect.x + BUTTON_PADDING
    y = rect.y + BUTTON_PADDING

    for idx, title in enumerate(self._menu_titles):
      btn_rect = rl.Rectangle(int(x), int(y), int(button_width), BUTTON_HEIGHT)

      is_pressed = rl.is_mouse_button_down(rl.MOUSE_LEFT_BUTTON) and rl.check_collision_point_rec(rl.get_mouse_position(), btn_rect)
      target_offset = 5 if is_pressed else 0
      self._press_offset_filter.update(target_offset)
      btn_rect.y += self._press_offset_filter.x

      shadow_offset = 2 if is_pressed else 4
      shadow_alpha = 180 if is_pressed else 120
      shadow_rect = rl.Rectangle(btn_rect.x + shadow_offset, btn_rect.y + shadow_offset, btn_rect.width, btn_rect.height)
      rl.draw_rectangle_rounded(shadow_rect, CORNER_RADIUS, 8, (0, 0, 0, shadow_alpha))

      base_color = HIGHLIGHT_COLOR if idx == self._current_menu else DEFAULT_COLOR
      if is_pressed:
          base_color = (max(base_color[0]-80,0), max(base_color[1]-80,0), max(base_color[2]-80,0), base_color[3])
      rl.draw_rectangle_rounded(btn_rect, CORNER_RADIUS, 8, base_color)

      text_width = rl.measure_text(title, TEXT_SIZE)
      text_x = int(x + (button_width - text_width) / 2)
      text_y = int(btn_rect.y + (BUTTON_HEIGHT - TEXT_SIZE) / 2)
      rl.draw_text(title, text_x, text_y, TEXT_SIZE, TEXT_COLOR)

      x += button_width + BUTTON_PADDING

    scroll_rect = rl.Rectangle(rect.x, rect.y + BUTTON_HEIGHT + BUTTON_PADDING * 2, rect.width, rect.height - (BUTTON_HEIGHT + BUTTON_PADDING * 2))
    self._scroller.render(scroll_rect)

  def show_event(self):
    self._scroller.show_event()
    self._update_items()

  def _update_items(self):
    ui_state.update_params()
    for key, item in self._param_mapping:
      if hasattr(item, "action_item"):
        action = item.action_item
        if getattr(action, "value_type", None) == "BOOL":
          action.set_state(self._params.get_bool(key))
    for menu_items in self._menu_items:
      for item in menu_items:
        meta = self._meta_mapping.get(item, {})

        visible_condition_can = not meta.get("can_type") or meta["can_type"] == self.can_type
        visible_condition_scc = not meta.get("scc_type") or meta["scc_type"] == self.scc_type

        item.set_visible(visible_condition_can and visible_condition_scc)

  def _show_car_selection_dialog(self):
    car_path = "/data/CarList"

    try:
      with open(car_path, "r") as f:
        car_files = [line.strip() for line in f.readlines() if line.strip()]
    except Exception:
      car_files = []

    if not car_files:
      gui_app.set_modal_overlay(
        alert_dialog("No car files found.")
      )
      return

    cur = self._params.get("CarSelected3")
    if isinstance(cur, (bytes, bytearray)):
      try:
        cur = cur.decode()
      except Exception:
        cur = str(cur)
    cur = cur or None

    def handle_car_selection(result: int):
      if result == 1 and self._select_car_dialog:
        selected_car = self._select_car_dialog.selection
        self._params.put("CarSelected3", selected_car)
        self._select_car_dialog = None
        return
      else:
        if not cur:
          self._select_car_dialog = None
          return
        else:
          def confirm_delete(res):
            if res == DialogResult.CONFIRM:
              self._params.remove("CarSelected3")
            self._select_car_dialog = None

          gui_app.set_modal_overlay(
            ConfirmDialog(f"Do you want to delete the current selection {cur} ?", "Delete"),
            callback=confirm_delete
          )
          self._select_car_dialog = None
          return

    self._select_car_dialog = MultiOptionDialog("Select Your Car", car_files, cur)
    gui_app.set_modal_overlay(self._select_car_dialog, callback=handle_car_selection)
