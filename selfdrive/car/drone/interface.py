import math
from cereal import car
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.car import get_safety_config
from openpilot.selfdrive.car.interfaces import CarInterfaceBase

class CarInterface(CarInterfaceBase):
  @staticmethod
  def _get_params(ret, candidate, fingerprint, car_fw, experimental_long, docs):
    ret.notCar = True
    ret.carName = "drone"
    ret.safetyConfigs = [get_safety_config(car.CarParams.SafetyModel.drone)]

    ret.minSteerSpeed = -math.inf
    ret.maxLateralAccel = math.inf  # TODO: set to a reasonable value
    ret.steerLimitTimer = 1.0
    ret.steerActuatorDelay = 0.1
    ret.autoResumeSng = True
    ret.stoppingControl = True

    ret.wheelSpeedFactor = 1.

    ret.radarUnavailable = True
    ret.openpilotLongitudinalControl = True
    ret.steerControlType = car.CarParams.SteerControlType.angle
    ret.vEgoStarting = 0.1
    ret.vEgoStopping = 0.1

    return ret

  def _update(self, c):
    ret = self.CS.update(self.cp)

    # wait for everything to init first
    if self.frame > int(5. / DT_CTRL):
      # body always wants to enable
      ret.init('events', 1)
      ret.events[0].name = car.CarEvent.EventName.pcmEnable
      ret.events[0].enable = True
    self.frame += 1

    return ret
