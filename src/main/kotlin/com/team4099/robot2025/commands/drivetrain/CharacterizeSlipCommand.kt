package com.team4099.robot2025.commands.drivetrain

import com.ctre.phoenix6.swerve.SwerveModule
import com.ctre.phoenix6.swerve.SwerveRequest
import com.team4099.lib.hal.Clock
import com.team4099.robot2025.subsystems.drivetrain.CommandSwerveDrive
import com.team4099.robot2025.util.CustomLogger
import edu.wpi.first.wpilibj2.command.Command
import org.team4099.lib.units.base.inSeconds
import org.team4099.lib.units.base.seconds

class CharacterizeSlipCommand(val drivetrain: CommandSwerveDrive): Command() {
  private val request = SwerveRequest.RobotCentric()
    .withDriveRequestType(SwerveModule.DriveRequestType.Velocity)
    .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo)

  private var startTime = 0.0.seconds

  init {
    addRequirements(drivetrain)

  }

  override fun initialize() {}

  override fun execute() {
    if (startTime == 0.0.seconds) startTime = Clock.fpgaTime

    val curSpeedMPS = (Clock.fpgaTime - startTime).inSeconds / 100

    drivetrain.setControl(
      request
        .withVelocityX(curSpeedMPS)
        .withVelocityY(0.0)
        .withRotationalRate(0.0)
    )

    CustomLogger.recordOutput("CharacterizeSlipCommand/curSpeed", curSpeedMPS)
  }
}