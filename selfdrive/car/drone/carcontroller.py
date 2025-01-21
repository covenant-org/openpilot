import math

from opendbc.can.packer import CANPacker
from openpilot.selfdrive.car.drone import bodycan
from openpilot.selfdrive.car.interfaces import CarControllerBase
from openpilot.selfdrive.controls.lib.pid import PIDController

MAX_ANGLE=math.radians(45)

class CarController(CarControllerBase):
  def __init__(self, dbc_name, CP, VM):
    self.frame = 0
    self.packer = CANPacker(dbc_name)

  def update(self, CC, CS, now_nanos):
    # [0.0, 1.0]
    accel = CC.actuators.accel
    desired_speed = (CS.out.vEgo * 0.97) + accel * 100
    if desired_speed > 10:
      desired_speed = 10
    if desired_speed < 0:
      desired_speed = 0

    w = 4.5
    steering_angle = math.radians(CC.actuators.steeringAngleDeg)
    if(abs(steering_angle) > MAX_ANGLE):
      steering_angle = math.copysign(MAX_ANGLE, steering_angle)
    turn = desired_speed * math.sin(steering_angle) / w
    turn_degrees = math.degrees(turn)

    can_sends = []
    print(CC.actuators)
    print(f"{int(turn_degrees * 10)} {int(desired_speed * 100)}")
    can_sends.append(bodycan.create_control(self.packer, turn_degrees, desired_speed))

    new_actuators = CC.actuators.as_builder()
    new_actuators.steerOutputCan = turn_degrees
    new_actuators.steeringAngleDeg = math.degrees(turn)
    new_actuators.speed = desired_speed
    new_actuators.accel = accel

    self.frame += 1
    return new_actuators, can_sends
