def create_control(packer, angle, speed):
  values = {
    "ANGLE": angle,
    "SPEED": speed,
  }

  return packer.make_can_msg("CMD", 0, values)
