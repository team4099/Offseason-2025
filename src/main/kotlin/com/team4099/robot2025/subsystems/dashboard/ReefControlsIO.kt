package com.team4099.robot2025.subsystems.dashboard

import org.littletonrobotics.junction.AutoLog

interface ReefControlsIO {
  @AutoLog
  class ReefControlsIOInputs {
    var coralState: BooleanArray = booleanArrayOf()
    var algaeState: BooleanArray = booleanArrayOf()
    var priorities: Array<String> = arrayOf()
  }

  fun updateInputs(inputs: ReefControlsIOInputs) {}

  fun setCoralState(value: BooleanArray) {}

  fun setAlgaeState(value: BooleanArray) {}

  fun setPriorities(value: Array<String>) {}
}
