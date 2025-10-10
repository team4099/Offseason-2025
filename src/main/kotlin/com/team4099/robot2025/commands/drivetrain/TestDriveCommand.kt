package com.team4099.robot2025.commands.drivetrain

import com.ctre.phoenix6.swerve.SwerveModule
import com.ctre.phoenix6.swerve.SwerveRequest
import com.team4099.robot2025.subsystems.drivetrain.CommandSwerveDrive
import edu.wpi.first.wpilibj2.command.Command

class TestDriveCommand(
  private val drivetrain: CommandSwerveDrive
): Command() {
  private val request = SwerveRequest.RobotCentric()
    .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
    .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo)

  init {
    addRequirements(drivetrain)
  }

  override fun initialize() {}

  override fun execute() {
    drivetrain.setControl(request.withVelocityX(1.0))
  }

  override fun end(interrupted: Boolean) {
    drivetrain.setControl(request.withVelocityX(0.0))
  }
}