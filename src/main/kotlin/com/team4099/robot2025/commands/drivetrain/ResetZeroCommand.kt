package com.team4099.robot2025.commands.drivetrain

import com.team4099.robot2025.subsystems.drivetrain.CommandSwerveDrive
import com.team4099.robot2025.util.CustomLogger
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj2.command.Command

class ResetZeroCommand(val drivetrain: CommandSwerveDrive) : Command() {
  init {
    addRequirements(drivetrain)
  }

  override fun initialize() {
    drivetrain.resetRotation(Rotation2d())
  }

  override fun execute() {
    CustomLogger.recordDebugOutput("ActiveCommands/ResetZeroCommand", true)
  }
}
