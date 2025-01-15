from cereal import car
from opendbc.can.parser import CANParser
from openpilot.selfdrive.car.interfaces import CarStateBase
from openpilot.selfdrive.car.drone.values import DBC

STARTUP_TICKS = 100

class CarState(CarStateBase):
  def update(self, cp):
    ret = car.CarState.new_message()
    print(cp.vl['DRONE_DATA'])

#    ret.wheelSpeeds.fl = cp.vl['MOTORS_DATA']['SPEED_L']
#    ret.wheelSpeeds.fr = cp.vl['MOTORS_DATA']['SPEED_R']
#
#    ret.vEgoRaw = ((ret.wheelSpeeds.fl + ret.wheelSpeeds.fr) / 2.) * self.CP.wheelSpeedFactor
#
#    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    ret.standstill = False

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
