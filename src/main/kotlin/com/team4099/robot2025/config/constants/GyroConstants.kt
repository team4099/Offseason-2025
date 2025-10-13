package com.team4099.robot2025.config.constants

import org.team4099.lib.units.derived.degrees

object GyroConstants {
  val mountPitch = 0.0.degrees
  val mountYaw = 0.0.degrees
  val mountRoll = 0.0.degrees

  val ANTI_TILT_THRESHOLD_PITCH = -180.degrees + 20.0.degrees
  val ANTI_TILT_THRESHOLD_ROLL = -180.degrees + 20.0.degrees
}
