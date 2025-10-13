package com.team4099.robot2025.commands.drivetrain

import com.ctre.phoenix6.swerve.SwerveModule
import com.ctre.phoenix6.swerve.SwerveRequest
import com.team4099.lib.math.asPose2d
import com.team4099.robot2025.config.constants.DrivetrainConstants
import com.team4099.robot2025.subsystems.drivetrain.Drive
import com.team4099.robot2025.subsystems.vision.Vision
import com.team4099.robot2025.util.CustomLogger
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.Command
import org.team4099.lib.controller.PIDController
import org.team4099.lib.geometry.Transform2d
import org.team4099.lib.geometry.Translation2d
import org.team4099.lib.kinematics.ChassisSpeeds
import org.team4099.lib.units.Velocity
import org.team4099.lib.units.base.Length
import org.team4099.lib.units.base.Meter
import org.team4099.lib.units.base.inInches
import org.team4099.lib.units.base.inches
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.Radian
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inDegrees
import org.team4099.lib.units.derived.radians
import kotlin.math.PI

class CoolerTargetTagCommand(
  private val drivetrain: Drive,
  private val vision: Vision,
  private val xTargetOffset: Length =
    DrivetrainConstants.DRIVETRAIN_LENGTH / 2 + DrivetrainConstants.BUMPER_WIDTH,
  private val yTargetOffset: Length = 0.0.inches,
  private val thetaTargetOffset: Angle = 0.0.radians,
) : Command() {

  private var thetaPID: PIDController<Radian, Velocity<Radian>> =
    PIDController(
      DrivetrainConstants.PID.TELEOP_THETA_PID_KP,
      DrivetrainConstants.PID.TELEOP_THETA_PID_KI,
      DrivetrainConstants.PID.TELEOP_THETA_PID_KD
    )
  private var yPID: PIDController<Meter, Velocity<Meter>> =
    PIDController(
      DrivetrainConstants.PID.TELEOP_Y_PID_KP,
      DrivetrainConstants.PID.TELEOP_Y_PID_KI,
      DrivetrainConstants.PID.TELEOP_Y_PID_KD
    )
  private var xPID: PIDController<Meter, Velocity<Meter>> =
    PIDController(
      DrivetrainConstants.PID.TELEOP_Y_PID_KP,
      DrivetrainConstants.PID.TELEOP_Y_PID_KI,
      DrivetrainConstants.PID.TELEOP_Y_PID_KD
    )

  private val requestRobotCentric =
    SwerveRequest.RobotCentric()
      .withDriveRequestType(SwerveModule.DriveRequestType.Velocity)
      .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo)

  init {
    addRequirements(drivetrain, vision)
    thetaPID.enableContinuousInput(-PI.radians, PI.radians)
  }

  override fun initialize() {
    CustomLogger.recordOutput("ActiveCommands/CoolerTargetTagCommand", true)

    if (RobotBase.isSimulation()) {
      thetaPID =
        PIDController(
          DrivetrainConstants.PID.SIM_AUTO_THETA_PID_KP,
          DrivetrainConstants.PID.SIM_AUTO_THETA_PID_KI,
          DrivetrainConstants.PID.SIM_AUTO_THETA_PID_KD
        )

      yPID =
        PIDController(
          DrivetrainConstants.PID.SIM_TELEOP_Y_PID_KP,
          DrivetrainConstants.PID.SIM_TELEOP_Y_PID_KI,
          DrivetrainConstants.PID.SIM_TELEOP_Y_PID_KD
        )

      xPID =
        PIDController(
          DrivetrainConstants.PID.SIM_TELEOP_Y_PID_KP,
          DrivetrainConstants.PID.SIM_TELEOP_Y_PID_KI,
          DrivetrainConstants.PID.SIM_TELEOP_Y_PID_KD
        )
    } else {
      if (DriverStation.isAutonomous()) {
        thetaPID =
          PIDController(
            DrivetrainConstants.PID.AUTO_REEF_PID_KP,
            DrivetrainConstants.PID.AUTO_REEF_PID_KI,
            DrivetrainConstants.PID.AUTO_REEF_PID_KD
          )
      } else {
        thetaPID =
          PIDController(
            DrivetrainConstants.PID.TELEOP_THETA_PID_KP,
            DrivetrainConstants.PID.TELEOP_THETA_PID_KI,
            DrivetrainConstants.PID.TELEOP_THETA_PID_KD
          )
      }

      yPID =
        PIDController(
          DrivetrainConstants.PID.TELEOP_Y_PID_KP,
          DrivetrainConstants.PID.TELEOP_Y_PID_KI,
          DrivetrainConstants.PID.TELEOP_Y_PID_KD
        )

      xPID =
        PIDController(
          DrivetrainConstants.PID.TELEOP_X_PID_KP,
          DrivetrainConstants.PID.TELEOP_X_PID_KI,
          DrivetrainConstants.PID.TELEOP_X_PID_KD
        )
    }

    xPID.reset()
    yPID.reset()
    thetaPID.reset()

    xPID.errorTolerance = 2.inches
    yPID.errorTolerance = 2.inches
    thetaPID.errorTolerance = 2.degrees

    vision.isAligned = false
  }

  override fun execute() {
    val odomTTag = vision.lastTrigVisionUpdate.robotTReefTag

    val exists = odomTTag != Transform2d(Translation2d(), 0.degrees)
    CustomLogger.recordOutput("CoolerTargetTagCommand/odomTTagExists", exists)
    if (!exists) return; // todo kalman?

    val setpointTranslation = odomTTag.translation
    val setpointRotation = odomTTag.rotation

    CustomLogger.recordOutput("CoolerTargetTagCommand/odomTTag", odomTTag.asPose2d().pose2d)
    CustomLogger.recordOutput(
      "CoolerTargetTagCommand/setpointTranslation", setpointTranslation.translation2d
    )

    // todo check signs and whatnot
    val xvel = xPID.calculate(setpointTranslation.x, xTargetOffset)
    val yvel = yPID.calculate(setpointTranslation.y, yTargetOffset)
    val thetavel = -thetaPID.calculate(setpointRotation, thetaTargetOffset)

    CustomLogger.recordOutput("CoolerTargetTagCommand/xerror", xPID.error.inInches)
    CustomLogger.recordOutput("CoolerTargetTagCommand/yerror", yPID.error.inInches)
    CustomLogger.recordOutput("CoolerTargetTagCommand/thetaerror", thetaPID.error.inDegrees)

    CustomLogger.recordOutput("CoolerTargetTagCommand/isAligned", isAtSetpoint())

    drivetrain.runVelocity(ChassisSpeeds(xvel, yvel, thetavel))

    CustomLogger.recordOutput("CoolerTargetTagCommand/isAtSetpoint", isAtSetpoint())

    if (isAtSetpoint()) vision.isAligned = true
  }

  override fun isFinished(): Boolean {
    return vision.isAligned
  }

  fun isAtSetpoint(): Boolean {
    return xPID.isAtSetpoint && yPID.isAtSetpoint && thetaPID.isAtSetpoint
  }

  override fun end(interrupted: Boolean) {
    drivetrain.runVelocity(ChassisSpeeds())
    CustomLogger.recordOutput("ActiveCommands/TargetTagCommand", false)
  }
}
