package com.team4099.robot2025.commands.drivetrain

import com.ctre.phoenix6.swerve.SwerveModule
import com.ctre.phoenix6.swerve.SwerveRequest
import com.team4099.robot2025.subsystems.drivetrain.CommandSwerveDrive
import com.team4099.robot2025.util.CustomLogger
import edu.wpi.first.wpilibj2.command.Command
import org.team4099.lib.geometry.Rotation2dWPILIB
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inRadians

class ResetGyroYawCommand(val drivetrain: CommandSwerveDrive, val toAngle: Angle = 0.0.degrees) :
  Command() {
  init {
    addRequirements(drivetrain)
  }

  override fun initialize() {
    drivetrain.resetRotation(Rotation2dWPILIB(toAngle.inRadians))
    drivetrain.setControl(
      SwerveRequest.PointWheelsAt()
        .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo)
        .withModuleDirection(Rotation2dWPILIB(toAngle.inRadians))
    )
  }

  override fun execute() {
    CustomLogger.recordDebugOutput("ActiveCommands/ResetGyroYawCommand", true)
  }

  override fun isFinished(): Boolean {
    return true
  }
}
