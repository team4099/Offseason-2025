package com.team4099.robot2025.subsystems.pneumatictest

import com.team4099.robot2025.config.constants.PneumaticConstants
import com.team4099.robot2025.util.CustomLogger
import edu.wpi.first.wpilibj2.command.SubsystemBase

class PneumaticTest(val io: PneumaticIO) : SubsystemBase() {
  var currentState: PneumaticTestState = PneumaticTestState.IDLE

  override fun periodic() {
    CustomLogger.recordOutput("Pneumatic/currentState", currentState.name)

    when (currentState) {
      PneumaticTestState.FORWARD -> io.setPower(PneumaticConstants.PneumaticOutputs.FORWARD)
      PneumaticTestState.REVERSE -> io.setPower(PneumaticConstants.PneumaticOutputs.REVERSE)
      else -> {}
    }
  }

  companion object {
    enum class PneumaticTestState {
      IDLE,
      FORWARD,
      REVERSE
    }
  }
}