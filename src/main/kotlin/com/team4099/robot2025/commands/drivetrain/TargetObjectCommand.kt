package com.team4099.robot2025.commands.drivetrain

import com.team4099.lib.hal.Clock
import com.team4099.robot2025.config.constants.Constants
import com.team4099.robot2025.config.constants.DrivetrainConstants
import com.team4099.robot2025.config.constants.VisionConstants
import com.team4099.robot2025.subsystems.drivetrain.Drive
import com.team4099.robot2025.subsystems.superstructure.Superstructure
import com.team4099.robot2025.subsystems.vision.Vision
import com.team4099.robot2025.util.CustomLogger
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.Command
import org.team4099.lib.controller.PIDController
import org.team4099.lib.geometry.Translation2d
import org.team4099.lib.kinematics.ChassisSpeeds
import org.team4099.lib.units.Value
import org.team4099.lib.units.Velocity
import org.team4099.lib.units.base.Time
import org.team4099.lib.units.base.inMeters
import org.team4099.lib.units.base.inSeconds
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.Radian
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inDegrees
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.inDegreesPerSecond
import org.team4099.lib.units.perSecond
import kotlin.math.PI

class TargetObjectCommand(
  private val drivetrain: Drive,
  private val vision: Vision,
  private val targetObjectClass: VisionConstants.OBJECT_CLASS,
  private val superstructure: Superstructure
) : Command() {
  private val thetaPID: PIDController<Radian, Velocity<Radian>>
  private var hasThetaAligned: Boolean = false

  private var startTime: Time = 0.0.seconds

  init {
    addRequirements(drivetrain, vision)

    if (RobotBase.isSimulation()) {
      thetaPID =
        PIDController(
          DrivetrainConstants.PID.SIM_AUTO_THETA_PID_KP,
          DrivetrainConstants.PID.SIM_AUTO_THETA_PID_KI,
          DrivetrainConstants.PID.SIM_AUTO_THETA_PID_KD
        )
    } else if (DriverStation.isAutonomous()) {
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
    thetaPID.enableContinuousInput(-PI.radians, PI.radians)
  }
  override fun initialize() {
    startTime = Clock.fpgaTime

    thetaPID.reset()

    thetaPID.enableContinuousInput(-PI.radians, PI.radians)

    vision.isAligned = false
    vision.isAutoAligning = true
    hasThetaAligned = false

    CustomLogger.recordOutput("TargetObjectCommand/lastInitialized", Clock.fpgaTime.inSeconds)
  }
  override fun execute() {
    CustomLogger.recordOutput("ActiveCommands/TargetObjectCommand", true)

    val lastUpdate = vision.lastObjectVisionUpdate[targetObjectClass.id]
    val robotTObject = lastUpdate.robotTObject

    val exists = (robotTObject != Translation2d())

    CustomLogger.recordOutput("TargetObjectCommand/odomTObjectExists", exists)
    if (!exists || Clock.realTimestamp - lastUpdate.timestamp > .2.seconds) end(interrupted = true)

    CustomLogger.recordOutput("TargetObjectCommand/odomTObjectx", robotTObject.x.inMeters)
    CustomLogger.recordOutput("TargetObjectCommand/odomTObjecty", robotTObject.y.inMeters)

    val setpointRotation: Value<Radian> =
      robotTObject.translation2d.angle.radians.radians +
        drivetrain
          .pose
          .rotation // atan2(-robotTObject.y.inMeters, -robotTObject.x.inMeters).radians

    CustomLogger.recordOutput("TargetObjectCommand/setPointRotation", setpointRotation.inDegrees)
    CustomLogger.recordOutput("TargetObjectCommand/driverot", drivetrain.rotation.inDegrees)

    var thetavel =
      thetaPID.calculate(drivetrain.pose.rotation, setpointRotation) *
        if (RobotBase.isReal()) -1.0 else 1.0

    CustomLogger.recordOutput("TargetObjectCommand/thetaveldps", thetavel.inDegreesPerSecond)
    CustomLogger.recordOutput("TargetObjectCommand/thetaerror", thetaPID.error.inDegrees)
    CustomLogger.recordOutput("TargetObjectCommand/hasThetaAligned", hasThetaAligned)

    if (hasThetaAligned || thetaPID.error.absoluteValue < 4.49.degrees) {
      hasThetaAligned = true

      drivetrain.runSpeeds(
        ChassisSpeeds(DrivetrainConstants.OBJECT_APPROACH_SPEED, 0.meters.perSecond, thetavel),
        flipIfRed = false
      )
    } else {
      drivetrain.runSpeeds(
        ChassisSpeeds(0.meters.perSecond, 0.meters.perSecond, thetavel), flipIfRed = false
      )
    }
  }
  override fun end(interrupted: Boolean) {
    if (!interrupted) vision.isAligned = true
    vision.isAutoAligning = false

    drivetrain.runSpeeds(ChassisSpeeds())
    CustomLogger.recordOutput("ActiveCommands/TargetObjectCommand", false)

    CustomLogger.recordOutput("TargetObjectCommand/interrupted", interrupted)
  }

  override fun isFinished(): Boolean {
    return if (targetObjectClass.name == "Coral") {
      superstructure.theoreticalGamePieceHardstop != Constants.Universal.GamePiece.NONE
    } else {
      superstructure.theoreticalGamePieceArm != Constants.Universal.GamePiece.NONE
    }
  }
}
