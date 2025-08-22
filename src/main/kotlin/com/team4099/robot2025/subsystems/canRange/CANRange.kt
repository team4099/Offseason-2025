package com.team4099.robot2025.subsystems.canRange

import com.team4099.lib.hal.Clock
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.base.Time

class CANRange(val io: CANRangeIO) : SubsystemBase() {
  val inputs = CANRangeIO.CANRangeIOInputs()
  var rumbleTrigger = false
  private var rumbleStartTime: Time = (-1337).seconds
  private var alreadyDetected = false

  override fun periodic() {
    io.updateInputs(inputs)

    if (inputs.isDetected && !alreadyDetected) {
      rumbleTrigger = true
      rumbleStartTime = Clock.fpgaTime
      alreadyDetected = true
    } else if (!inputs.isDetected) {
      rumbleTrigger = false
      alreadyDetected = false
    }

    if (rumbleTrigger && (Clock.fpgaTime - rumbleStartTime > 0.5.seconds)) {
      rumbleTrigger = false
    }
  }
}

