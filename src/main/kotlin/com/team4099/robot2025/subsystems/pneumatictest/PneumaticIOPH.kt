package com.team4099.robot2025.subsystems.pneumatictest

import com.team4099.robot2025.config.constants.PneumaticConstants
import edu.wpi.first.wpilibj.Compressor
import edu.wpi.first.wpilibj.DoubleSolenoid
import edu.wpi.first.wpilibj.PneumaticHub

object PneumaticIOPH : PneumaticIO {
  private val ph = PneumaticHub()
  private val solenoid: DoubleSolenoid = ph.makeDoubleSolenoid(0, 1)
  private val compressor: Compressor = ph.makeCompressor()

  init {
    ph.enableCompressorAnalog(80.0, 120.0)
  }

  override fun setPower(output: PneumaticConstants.PneumaticOutputs) {
    solenoid.set(
      when (output) {
        PneumaticConstants.PneumaticOutputs.FORWARD -> DoubleSolenoid.Value.kForward
        PneumaticConstants.PneumaticOutputs.REVERSE -> DoubleSolenoid.Value.kReverse
      }
    )
  }
}