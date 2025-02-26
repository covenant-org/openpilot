import math

import os
from openpilot.common.realtime import DT_CTRL
from opendbc.can.packer import CANPacker
from openpilot.selfdrive.car.drone import bodycan
from openpilot.selfdrive.car.interfaces import CarControllerBase
from openpilot.selfdrive.controls.lib.pid import PIDController

MAX_ANGLE=45

class CarController(CarControllerBase):
  def __init__(self, dbc_name, CP, VM):
    self.frame = 0
    self.packer = CANPacker(dbc_name)
    self.height_target = float(os.getenv("DRONE_HEIGHT", 1.0))
    self.altitude_pid = PIDController(0.65, k_i=0.55, rate=1/DT_CTRL)

  def update(self, CC, CS, now_nanos):
    # [0.0, 1.0]
    accel = CC.actuators.accel
    desired_speed = (CS.out.vEgo) + accel
    if desired_speed > 3:
      desired_speed = 3
    if desired_speed < 0:
      desired_speed = 0

    w = 4.5
    steering_angle_deg = CC.actuators.steeringAngleDeg * MAX_ANGLE
    if(abs(steering_angle_deg) > MAX_ANGLE):
      steering_angle_deg = MAX_ANGLE
    steering_angle = math.radians(steering_angle_deg)
    turn = desired_speed * math.sin(steering_angle) / w
    turn_degrees = math.degrees(turn)

    can_sends = []
    #print(CC.actuators)
    # TODO:proper altitude param in car state
    altitude = CS.out.wheelSpeeds.rl
    target = CS.out.wheelSpeeds.rr
    if target != self.height_target:
      self.altitude_pid.reset()
    self.height_target = target
    altitude_error = self.height_target - altitude
    down_ms = self.altitude_pid.update(altitude_error, freeze_integrator=False)
    can_sends.append(bodycan.create_control(self.packer, turn_degrees, desired_speed, down_ms))

    new_actuators = CC.actuators.as_builder()
    new_actuators.steerOutputCan = turn_degrees
    new_actuators.steeringAngleDeg = math.degrees(turn)
    new_actuators.speed = desired_speed

    self.frame += 1
    return new_actuators, can_sends
