package com.team4099.robot2025.subsystems.superstructure.canRange

import com.team4099.lib.hal.Clock
import com.team4099.robot2025.util.ControlledByStateMachine
import com.team4099.robot2025.util.CustomLogger
import org.team4099.lib.units.base.seconds

class CANRange(val io: CANRangeIO) : ControlledByStateMachine() {
  val inputs = CANRangeIO.CANRangeIOInputs()
  var rumbleTrigger = false
  var rumbleTime = (-1337).seconds
  var hasRumbledForThisDetection = false

  override fun onLoop() {
    io.updateInputs(inputs)

    CustomLogger.processInputs("CANRange", inputs)
    CustomLogger.recordOutput("CANRange/rumbleTrigger", rumbleTrigger)

    if (inputs.isDetected) {
      if (!hasRumbledForThisDetection) {
        rumbleTrigger = true
        rumbleTime = Clock.fpgaTime
        hasRumbledForThisDetection = true
      } else {
        rumbleTrigger = (Clock.fpgaTime - rumbleTime) < 0.5.seconds
      }
    } else {
      hasRumbledForThisDetection = false
      rumbleTrigger = false
    }
  }
}
