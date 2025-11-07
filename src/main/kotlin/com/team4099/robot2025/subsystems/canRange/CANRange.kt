package com.team4099.robot2025.subsystems.canRange

import com.team4099.lib.hal.Clock
import com.team4099.robot2025.util.CustomLogger
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.team4099.lib.units.base.inMilliseconds
import org.team4099.lib.units.base.seconds

class CANRange(val io: CANRangeIO) : SubsystemBase() {
  val inputs = CANRangeIO.CANRangeIOInputs()
  var rumbleTrigger = false
  var rumbleTime = (-1337).seconds
  var hasRumbledForThisDetection = false

  override fun periodic() {
    val startTime = Clock.fpgaTime

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

    CustomLogger.recordOutput(
      "LoggedRobot/Subsystems/CANRangeLoopTimeMS", (Clock.fpgaTime - startTime).inMilliseconds
    )
  }
}
