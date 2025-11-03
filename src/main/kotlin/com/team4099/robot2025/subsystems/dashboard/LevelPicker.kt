package com.team4099.robot2025.subsystems.dashboard

import com.team4099.robot2025.commands.drivetrain.CoolerTargetTagCommand
import com.team4099.robot2025.util.CustomLogger
import com.team4099.robot2025.util.VirtualSubsystem

class LevelPicker(val io: ReefControlsIO) : VirtualSubsystem() {

 val inputs = ReefControlsIO.ReefControlsIOInputs()
  val x = CoolerTargetTagCommand.LastAlignedBranch

  override fun periodic() {
    io.updateInputs(inputs)
    CustomLogger.recordOutput("Dashboard/ReefState/Coral", inputs.coralState)
    CustomLogger.recordOutput("Dashboard/ReefState/Algae", inputs.algaeState)
    CustomLogger.recordOutput("Dashboard/ReefState/Priorities", inputs.priorities)


  }
}