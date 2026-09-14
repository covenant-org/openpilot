#!/usr/bin/env python3
import numpy as np

from openpilot.common.pid import PIDController


class FanController:
  def __init__(self, rate: int) -> None:
    self.last_ignition = False
    self.controller = PIDController(k_p=0, k_i=4e-3, rate=rate)

  def update(self, cur_temp: float, ignition: bool) -> int:
    # Offroad the fan ceiling used to be a flat 30%, to spare the battery on the
    # assumption that parked means cool and idle. That assumption inverts on a
    # truck parked in direct sun. Measured 2026-08-04 on a Kenworth T680 in
    # Guadalajara: the device sat at 83-88 C for a full hour with the engine
    # off, ZERO recording load and flat CPU load, while this controller held
    # the fan at 30% purely because ignition was false. Heat was entirely
    # external; the device had no way to shed it.
    #
    # hardwared's own OFFROAD_DANGER_TEMP is 75 C — above that it already
    # refuses to go onroad "to cool down first". Pinning the fan at 30%
    # throughout that window works directly against what that check is for.
    #
    # So ramp the offroad ceiling with temperature rather than pinning it:
    # unchanged at 30% up to 78 C (ordinary parked — battery preserved exactly
    # as before), rising to full only as the device approaches the red band
    # (88 C) and danger band (94 C). Battery cost scales with actual need
    # instead of being paid always or never.
    #
    # NOTE this is a real trade: more fan while parked is more current draw,
    # and this truck load-sheds on battery voltage. The ramp keeps the cost at
    # zero in normal conditions and spends only when the alternative is a
    # thermal shutdown, which is what a dead device costs instead.
    offroad_cap = float(np.interp(cur_temp, [78.0, 92.0], [30.0, 100.0]))
    self.controller.pos_limit = 100 if ignition else offroad_cap
    self.controller.neg_limit = 30 if ignition else 0

    if ignition != self.last_ignition:
      self.controller.reset()
    self.last_ignition = ignition

    return int(self.controller.update(
                 error=(cur_temp - 75),  # temperature setpoint in C
                 feedforward=np.interp(cur_temp, [60.0, 100.0], [0, 100])
              ))
