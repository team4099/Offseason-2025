package com.team4099.robot2025.commands.drivetrain

import com.ctre.phoenix6.swerve.SwerveModule
import com.ctre.phoenix6.swerve.SwerveRequest
import com.team4099.lib.hal.Clock
import com.team4099.lib.math.asPose2d
import com.team4099.robot2025.config.constants.DrivetrainConstants
import com.team4099.robot2025.subsystems.drivetrain.CommandSwerveDrive
import com.team4099.robot2025.subsystems.vision.Vision
import com.team4099.robot2025.util.CustomLogger
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.Command
import org.team4099.lib.controller.PIDController
import org.team4099.lib.geometry.Transform2d
import org.team4099.lib.geometry.Translation2d
import org.team4099.lib.units.Velocity
import org.team4099.lib.units.base.Length
import org.team4099.lib.units.base.Meter
import org.team4099.lib.units.base.inInches
import org.team4099.lib.units.base.inSeconds
import org.team4099.lib.units.base.inches
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.centi
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.Radian
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inDegrees
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.inDegreesPerSecond
import org.team4099.lib.units.inMetersPerSecond
import org.team4099.lib.units.inRadiansPerSecond
import org.team4099.lib.units.perSecond
import kotlin.math.PI
import org.team4099.lib.units.derived.inRotation2ds

class CoolerTargetTagCommand(
  private val drivetrain: CommandSwerveDrive,
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
      .withDeadband(0.5.centi.meters.perSecond.inMetersPerSecond)
//      .withRotationalDeadband(0.5.degrees.perSecond.inRadiansPerSecond)

  private val requestPointWheels =
    SwerveRequest.PointWheelsAt().withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo)
  .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)

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

    xPID.errorTolerance = .5.inches
    yPID.errorTolerance = .5.inches
    thetaPID.errorTolerance = .5.degrees

    vision.isAligned = false

    CustomLogger.recordOutput("CoolerTargetTagCommand/lastInitialized", Clock.fpgaTime.inSeconds)
  }

  override fun execute() {
    val odomTTag = vision.lastTrigVisionUpdate.robotTReefTag

    val exists = odomTTag != Transform2d(Translation2d(), 0.degrees)
    CustomLogger.recordOutput("CoolerTargetTagCommand/odomTTagExists", exists)
    if (!exists || Clock.realTimestamp - vision.lastTrigVisionUpdate.timestamp > 3.seconds)
      return; // todo kalman?

    val setpointTranslation = odomTTag.translation
    val setpointRotation = odomTTag.rotation

    CustomLogger.recordOutput("CoolerTargetTagCommand/odomTTag", odomTTag.asPose2d().pose2d)
    CustomLogger.recordOutput(
      "CoolerTargetTagCommand/expectedTagPose",
      drivetrain.state.Pose.transformBy(odomTTag.transform2d)
    )
    CustomLogger.recordOutput(
      "CoolerTargetTagCommand/setpointTranslation", setpointTranslation.translation2d
    )
    CustomLogger.recordOutput("CoolerTargetTagCommand/setpointRotation", setpointRotation.inDegrees)

    // todo check signs and whatnot
    var xvel = -xPID.calculate(setpointTranslation.x, xTargetOffset * setpointTranslation.x.sign)
    var yvel = -yPID.calculate(setpointTranslation.y, yTargetOffset)
    var thetavel = thetaPID.calculate(setpointRotation, thetaTargetOffset)

    CustomLogger.recordOutput("CoolerTargetTagCommand/xvelmps", xvel.inMetersPerSecond)
    CustomLogger.recordOutput("CoolerTargetTagCommand/yvelmps", yvel.inMetersPerSecond)
    CustomLogger.recordOutput("CoolerTargetTagCommand/thetaveldps", thetavel.inDegreesPerSecond)
    CustomLogger.recordOutput("CoolerTargetTagCommand/xerror", xPID.error.inInches)
    CustomLogger.recordOutput("CoolerTargetTagCommand/yerror", yPID.error.inInches)
    CustomLogger.recordOutput("CoolerTargetTagCommand/thetaerror", thetaPID.error.inDegrees)

    if (thetaPID.error.absoluteValue < 2.056.degrees && thetavel.absoluteValue < 0.5.degrees.perSecond) {
      drivetrain.setControl(
        requestPointWheels.withModuleDirection((setpointRotation - thetaTargetOffset).inRotation2ds)
      )
      drivetrain.setControl(
        requestRobotCentric
          .withVelocityX(xvel.inMetersPerSecond)
          .withVelocityY(yvel.inMetersPerSecond)
          .withRotationalRate(thetavel.inRadiansPerSecond)
      )
    }
    else {
      drivetrain.setControl(requestRobotCentric.withRotationalRate(thetavel.inRadiansPerSecond))
    }
  }

  override fun isFinished(): Boolean {
    return xPID.error < xPID.errorTolerance &&
      yPID.error < yPID.errorTolerance &&
      thetaPID.error < thetaPID.errorTolerance
  }

  override fun end(interrupted: Boolean) {
    if (!interrupted) vision.isAligned = true

    CustomLogger.recordOutput("CoolerTargetTagCommand/interrupted", interrupted)

    drivetrain.setControl(
      requestRobotCentric
        .withVelocityX(0.0)
        .withVelocityY(0.0)
        .withRotationalRate(0.0)
        .withDeadband(0.0)
        .withRotationalDeadband(0.0)
    )
    CustomLogger.recordOutput("ActiveCommands/TargetTagCommand", false)
  }
}
