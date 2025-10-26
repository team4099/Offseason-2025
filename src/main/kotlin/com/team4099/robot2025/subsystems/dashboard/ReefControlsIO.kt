package com.team4099.robot2025.subsystems.dashboard
import org.littletonrobotics.junction.AutoLog

interface ReefControlsIO {
  @AutoLog
  class ReefControlsIOInputs {
    var coralState: BooleanArray = booleanArrayOf() // Bitfield
    var algaeState: BooleanArray= booleanArrayOf() // Bitfield
  }

  fun updateInputs(inputs: ReefControlsIOInputs) {}

  fun setCoralState(value: Boolean) {}

  fun setAlgaeState(value: Boolean) {}
}