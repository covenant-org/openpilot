from cereal import car
from opendbc.can.parser import CANParser
from openpilot.selfdrive.car.interfaces import CarStateBase
from openpilot.selfdrive.car.drone.values import DBC

STARTUP_TICKS = 100

class CarState(CarStateBase):
  def update(self, cp):
    ret = car.CarState.new_message()

    ret.vEgoRaw = cp.vl['DRONE_DATA']['SPEED']
    ret.cruiseState.speed = cp.vl['DRONE_DATA']['SPEED']
#
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    ret.standstill = False
    ret.steerFaultPermanent = False

    # irrelevant for non-car
    ret.gearShifter = car.CarState.GearShifter.drive
    ret.cruiseState.enabled = True
    ret.cruiseState.available = True

    return ret

  @staticmethod
  def get_can_parser(CP):
    messages = [
      ("DRONE_DATA", 100),
    ]
    return CANParser(DBC[CP.carFingerprint]["pt"], messages, 0)
