package com.team4099.robot2025.subsystems.canRange

import edu.wpi.first.wpilibj2.command.SubsystemBase

class CANRange(val io: CANRangeIO) : SubsystemBase() {
  private val inputs = CANRangeIO.CANRangeIOInputs()

  override fun periodic() {
    io.updateInputs(inputs)
  }
}
