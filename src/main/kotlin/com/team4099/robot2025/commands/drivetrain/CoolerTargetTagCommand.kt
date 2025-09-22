package com.team4099.robot2025.commands.drivetrain

import com.ctre.phoenix6.swerve.SwerveModule
import com.ctre.phoenix6.swerve.SwerveRequest
import com.team4099.robot2025.config.constants.DrivetrainConstants
import com.team4099.robot2025.config.constants.VisionConstants
import com.team4099.robot2025.subsystems.drivetrain.CommandSwerveDrive
import com.team4099.robot2025.subsystems.vision.Vision
import com.team4099.robot2025.util.CustomLogger
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.Command
import org.team4099.lib.controller.PIDController
import org.team4099.lib.units.Velocity
import org.team4099.lib.units.base.Length
import org.team4099.lib.units.base.Meter
import org.team4099.lib.units.base.inches
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.Radian
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.inDegreesPerSecond
import org.team4099.lib.units.inMetersPerSecond
import kotlin.math.PI

class CoolerTargetTagCommand(
  private val drivetrain: CommandSwerveDrive,
  private val vision: Vision,
  private val xTargetOffset: Length = 0.meters,
  private val yTargetOffset: Length = 0.meters,
  private val thetaTargetOffset: Angle = 0.0.radians,
  private val flushX: Boolean = true,
  private val flushY: Boolean = false
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
          DrivetrainConstants.PID.TELEOP_Y_PID_KP,
          DrivetrainConstants.PID.TELEOP_Y_PID_KI,
          DrivetrainConstants.PID.TELEOP_Y_PID_KD
        )
    }

    xPID.reset()
    yPID.reset()
    thetaPID.reset()

    xPID.errorTolerance = 2.inches
    yPID.errorTolerance = 2.inches
    thetaPID.errorTolerance = 3.degrees

    vision.isAligned = false
  }

  override fun execute() {
    //    val (camTransform, odomTTag) = vision.shortestOdomTTagCamTransformAndTransform
    val camTransform = VisionConstants.CAMERA_TRANSFORMS[0]
    val odomTTag = vision.lastTrigVisionUpdate.robotTReefTag

    CustomLogger.recordOutput("CoolerTargetTagCommand/camTransformExists", camTransform != null)
    CustomLogger.recordOutput("CoolerTargetTagCommand/odomTTagExists", odomTTag != null)

    if (camTransform == null || odomTTag == null) return; // todo kalman?

    val setpointTranslation = odomTTag.translation
    val setpointRotation = odomTTag.rotation

    val xvel =
      xPID.calculate(
        -(setpointTranslation.x + camTransform.inverse().x),
        if (flushX) 3.25.inches else xTargetOffset
      )
    val yvel =
      yPID.calculate(
        -(setpointTranslation.y + camTransform.inverse().y),
        if (flushY) 3.25.inches else yTargetOffset
      )
    val thetavel =
      //      thetaPID.calculate(-(setpointRotation.z - camTransform.rotation.z), thetaTargetOffset)
      thetaPID.calculate(-(setpointRotation - camTransform.rotation.z), thetaTargetOffset)

    CustomLogger.recordOutput("CoolerTargetTagCommand/xvel", xvel.inMetersPerSecond)
    CustomLogger.recordOutput("CoolerTargetTagCommand/yvel", yvel.inMetersPerSecond)
    CustomLogger.recordOutput("CoolerTargetTagCommand/thetavel", thetavel.inDegreesPerSecond)

    //    if (thetaPID.error.absoluteValue > 5.degrees) {
    //      drivetrain.setControl(
    //        requestRobotCentric
    //          //          .withVelocityX(xvel.inMetersPerSecond)
    //          //          .withVelocityY(yvel.inMetersPerSecond)
    //          .withRotationalRate(thetavel.inRadiansPerSecond)
    //      )
    //    } else {
    drivetrain.setControl(
      requestRobotCentric
        .withVelocityX(xvel.inMetersPerSecond)
        .withVelocityY(yvel.inMetersPerSecond)
      //          .withRotationalRate(thetavel.inRadiansPerSecond)
    )
    //    }

    if (isAtSetpoint()) vision.isAligned = true
  }

  override fun isFinished(): Boolean {
    return false
  }

  fun isAtSetpoint(): Boolean {
    return xPID.isAtSetpoint && yPID.isAtSetpoint && thetaPID.isAtSetpoint
  }

  override fun end(interrupted: Boolean) {
    CustomLogger.recordOutput("ActiveCommands/TargetTagCommand", false)
  }
}
