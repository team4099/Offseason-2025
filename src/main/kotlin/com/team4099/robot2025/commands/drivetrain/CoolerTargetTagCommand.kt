package com.team4099.robot2025.commands.drivetrain

import com.ctre.phoenix6.swerve.SwerveModule
import com.ctre.phoenix6.swerve.SwerveRequest
import com.team4099.robot2025.config.constants.DrivetrainConstants
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
import org.team4099.lib.units.inMetersPerSecond
import org.team4099.lib.units.inRadiansPerSecond
import kotlin.math.PI

class CoolerTargetTagCommand(
  val drivetrain: CommandSwerveDrive, val vision: Vision, val yTargetOffset: Length = 0.meters, val thetaTargetOffset: Angle = 0.0.radians
) : Command() {

  private lateinit var thetaPID: PIDController<Radian, Velocity<Radian>>
  private lateinit var yPID: PIDController<Meter, Velocity<Meter>>
  private lateinit var xPID: PIDController<Meter, Velocity<Meter>>

  private val requestRobotCentric =
    SwerveRequest.RobotCentric()
      .withDriveRequestType(SwerveModule.DriveRequestType.Velocity)
      .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo)

  init {
    thetaPID.enableContinuousInput(-PI.radians, PI.radians)
  }

  override fun initialize() {
    CustomLogger.recordOutput("ActiveCommands/TargetTagCommand", true)

    if (RobotBase.isSimulation()) {
      thetaPID = PIDController(
        DrivetrainConstants.PID.SIM_AUTO_THETA_PID_KP,
        DrivetrainConstants.PID.SIM_AUTO_THETA_PID_KI,
        DrivetrainConstants.PID.SIM_AUTO_THETA_PID_KD
      )

      yPID = PIDController(
        DrivetrainConstants.PID.SIM_TELEOP_Y_PID_KP,
        DrivetrainConstants.PID.SIM_TELEOP_Y_PID_KI,
        DrivetrainConstants.PID.SIM_TELEOP_Y_PID_KD
      )

      xPID = PIDController(
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

  var x = null

  override fun execute() {
    val (camTransform, odomTTag) = vision.shortestOdomTTagCamTransformAndTransform

    if (camTransform == null || odomTTag == null) return; // todo kalman?

    val setpointTranslation = odomTTag.translation
    val setpointRotation = odomTTag.rotation


    val xvel = xPID.calculate(-(setpointTranslation.x + camTransform.inverse().x), DrivetrainConstants.DRIVETRAIN_LENGTH / 2 + 3.25.inches)
    val yvel = yPID.calculate(-(setpointTranslation.y + camTransform.inverse().y), yTargetOffset)
    val thetavel = thetaPID.calculate(-(setpointRotation.z - camTransform.rotation.z), thetaTargetOffset)

    if (thetaPID.error.absoluteValue > 5.degrees) {
      drivetrain.setControl(
        requestRobotCentric
//          .withVelocityX(xvel.inMetersPerSecond)
//          .withVelocityY(yvel.inMetersPerSecond)
          .withRotationalRate(thetavel.inRadiansPerSecond)
      )
    } else {
      drivetrain.setControl(
        requestRobotCentric
          .withVelocityX(xvel.inMetersPerSecond)
          .withVelocityY(yvel.inMetersPerSecond)
          .withRotationalRate(thetavel.inRadiansPerSecond)
      )
    }
  }

  override fun isFinished(): Boolean {
    return xPID.isAtSetpoint && yPID.isAtSetpoint && thetaPID.isAtSetpoint
  }

  override fun end(interrupted: Boolean) {
    CustomLogger.recordOutput("ActiveCommands/TargetTagCommand", false)
  }

}