def create_control(packer, angle, speed, down):
  values = {
    "ANGLE": angle,
    "SPEED": speed,
    "DOWN": down,
  }

  return packer.make_can_msg("CMD", 0, values)
