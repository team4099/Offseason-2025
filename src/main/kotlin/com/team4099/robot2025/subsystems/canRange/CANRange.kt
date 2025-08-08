package com.team4099.robot2025.subsystems.canRange

import com.team4099.lib.hal.Clock
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.team4099.lib.units.base.seconds

class CANRange(val io: CANRangeIO) : SubsystemBase() {
  private val inputs = CANRangeIO.CANRangeIOInputs()
  var rumbleTrigger = false
  var rumbleTime = (-1337).seconds

  override fun periodic() {
    io.updateInputs(inputs)

    if (inputs.isDetected && (!rumbleTrigger || Clock.fpgaTime - rumbleTime < 0.5.seconds)) {
      rumbleTrigger = true
      rumbleTime = Clock.fpgaTime
    } else if (!inputs.isDetected) {
      rumbleTrigger = false
    }
  }
}
