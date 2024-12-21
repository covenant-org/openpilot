# ruff: noqa: E501
from cereal import car
from openpilot.selfdrive.car.drone.values import CAR

Ecu = car.CarParams.Ecu

# debug ecu fw version is the git hash of the firmware


FINGERPRINTS = {
  CAR.DRONE: [{
    613: 8, 614: 4
  }],
}

FW_VERSIONS = {
  CAR.DRONE: {
    (Ecu.engine, 0x722, None): [
      b'ncl.0.01',
    ],
  },
}
