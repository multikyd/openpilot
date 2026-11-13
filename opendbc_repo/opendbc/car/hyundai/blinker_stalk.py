"""Physical Ioniq 5 PE lever from the classic 0x3C1 frame."""
STALK_ADDR = 0x3C1
STALK_MAX_AGE_NS = 350_000_000


def stalk_checksum(payload):
  crc = 0
  for byte in payload[1:]:
    crc ^= byte
    for _ in range(8):
      crc = ((crc << 1) ^ (0x1D if crc & 0x80 else 0)) & 0xFF
  return crc ^ 0x65


class BlinkerStalk:
  def __init__(self, bus):
    self.bus = bus
    self.stamp = 0
    self.left_until = self.right_until = 0

  def update(self, can_packets):
    if not can_packets:
      return False, False, False
    for stamp, frames in can_packets:
      for address, data, src in frames:
        if src != self.bus or address != STALK_ADDR or len(data) != 8:
          continue
        if data[0] != stalk_checksum(data):
          self.stamp = self.left_until = self.right_until = 0
          continue
        self.stamp = stamp
        left = bool(data[3] & 0x40 or data[4] & 0x10)
        right = bool(data[4] & 0x41)
        if left or right:
          self.left_until = stamp + STALK_MAX_AGE_NS if left else 0
          self.right_until = stamp + STALK_MAX_AGE_NS if right else 0
    now = can_packets[-1][0]
    valid = self.stamp > 0 and 0 <= now - self.stamp <= STALK_MAX_AGE_NS
    return valid, valid and now < self.left_until, valid and now < self.right_until
