package com.team4099.robot2025.config.constants

object PneumaticConstants {
  enum class PneumaticOutputs(val solenoidIndex: Int) {
    FORWARD(0),
    REVERSE(1)
  }
}