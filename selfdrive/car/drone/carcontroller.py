import math

from opendbc.can.packer import CANPacker
from openpilot.selfdrive.car.drone import bodycan
from openpilot.selfdrive.car.interfaces import CarControllerBase
from openpilot.selfdrive.controls.lib.pid import PIDController


class CarController(CarControllerBase):
  def __init__(self, dbc_name, CP, VM):
    self.frame = 0
    self.packer = CANPacker(dbc_name)

  def update(self, CC, CS, now_nanos):
    # [0.0, 1.0]
    gas = CC.actuators.gas
    brk = CC.actuators.brake
    accel = (gas - brk) * 0.5
    desired_speed = (CC.actuators.speed * 0.97) + accel
    if desired_speed > 3:
      desired_speed = 3

    w = 4.5
    steering_angle = CC.actuators.steer * (math.pi / 4)
    turn = desired_speed * math.sin(steering_angle) / w
    turn_degrees = math.degrees(turn)

    can_sends = []
    print(CC.actuators)
    print(f"{int(turn_degrees * 10)} {int(desired_speed * 100)}")
    can_sends.append(bodycan.create_control(self.packer, int(turn_degrees * 10), int(desired_speed * 100)))

    new_actuators = CC.actuators.as_builder()
    new_actuators.steerOutputCan = turn_degrees
    new_actuators.steeringAngleDeg = math.degrees(turn)
    new_actuators.speed = desired_speed
    new_actuators.accel = accel

    self.frame += 1
    return new_actuators, can_sends
