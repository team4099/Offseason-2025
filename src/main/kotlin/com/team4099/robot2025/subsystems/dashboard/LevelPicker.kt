package com.team4099.robot2025.subsystems.dashboard

import com.team4099.robot2025.util.VirtualSubsystem

class LevelPicker(val io: ReefControlsIO) : VirtualSubsystem() {

 val inputs = ReefControlsIO.ReefControlsIOInputs()


  override fun periodic() {
    io.updateInputs(inputs)
  }
}