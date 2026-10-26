# Brake-light model

Source: Shantanu Maity, Brake Light Detection, version 4 (YOLOv8s).
https://universe.roboflow.com/shantanu-maity-0wthp/brake-light-detection-jee6j

Project advertises CC BY 4.0: https://creativecommons.org/licenses/by/4.0/
YOLOv8 architecture/runtime upstream: https://github.com/ultralytics/ultralytics
The Roboflow inference-models Python package is not bundled on the vehicle.

Classes: 0 Light_OFF, 1 Light_ON. Boxes describe whole rear vehicles.
Input: float32 RGB [1,3,640,640], stretch, /255, full-range UI NV12 conversion.
Output: [1,6,8400], raw YOLOv8 xywh + two class scores, class-agnostic NMS.
Original weights MD5: bbd21aa31e635ace544e53cf80ab3ec1.
Weights copied unchanged from the authenticated local download.

Display-only integration: the fast temporal lamp detector retains priority over
model OFF on a fresh lamp onset; fresh high-confidence model ON/OFF is a fallback.
Conflicts and uncertainty retain a cyan detected-vehicle outline; cyan does not assert brake-OFF. Only whole vehicle outlines are drawn;
individual lamp locations remain internal label metadata. No control inputs.

## Lightweight local derivatives

brake_light_v4_320.onnx uses the same weights with a 320x320 input, regenerated
stride-8/16/32 anchor grids and matching 2100-candidate detection-head shapes.
brake_light_v4_320_int8.onnx is a calibrated per-channel Conv S8S8 QDQ derivative;
final detection heads remain floating point. Calibration used 20 existing local
older-drive images; test40 was held out. Dataset/architecture attribution above
also applies to both derivatives. Conversion/evaluation scripts are saved in the
local task output directory; no footage was uploaded for conversion.

Auto mode benchmarks both 320 variants on the device with dummy inputs and
prefers FP32 on a near tie. Unsupported INT8 falls back to FP32. Setting
LEAD_BRAKE_VARIANT=legacy selects the unchanged 640 model; fp32 or int8 selects
a candidate explicitly. No additional vehicle-side package is required beyond
the existing isolated ONNX Runtime bundle.

Reduced resolution and quantization can miss vehicles or change brake states.
Lightweight state hints therefore require confidence >=0.9 (vehicle outlines
retain >=0.65 plus radar/model association). Fast lamp onset/release evidence
and stale-result/generation protections remain authoritative.
