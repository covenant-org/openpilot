import numpy as np

from openpilot.common.realtime import DT_CTRL
from opendbc.can.packer import CANPacker
from openpilot.selfdrive.car.drone import bodycan
from openpilot.selfdrive.car.interfaces import CarControllerBase
from openpilot.selfdrive.controls.lib.pid import PIDController


class CarController(CarControllerBase):
  def __init__(self, dbc_name, CP, VM):
    self.frame = 0
    self.packer = CANPacker(dbc_name)

  def update(self, CC, CS, now_nanos):

    can_sends = []
    can_sends.append(bodycan.create_control(self.packer, 0, 0))

    new_actuators = CC.actuators.as_builder()

    self.frame += 1
    return new_actuators, can_sends
