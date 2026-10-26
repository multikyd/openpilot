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
Conflicts and uncertainty display gray. Only whole vehicle outlines are drawn;
individual lamp locations remain internal label metadata. No control inputs.
