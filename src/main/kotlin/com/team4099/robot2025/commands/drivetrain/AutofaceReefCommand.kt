package com.team4099.robot2025.commands.drivetrain

import com.team4099.robot2025.config.constants.Constants
import com.team4099.robot2025.config.constants.DrivetrainConstants
import com.team4099.robot2025.subsystems.drivetrain.Drive
import com.team4099.robot2025.util.CustomLogger
import com.team4099.robot2025.util.driver.DriverProfile
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.Command
import org.team4099.lib.controller.PIDController
import org.team4099.lib.geometry.Pose2d
import org.team4099.lib.kinematics.ChassisSpeeds
import org.team4099.lib.units.Velocity
import org.team4099.lib.units.base.inMeters
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.Radian
import org.team4099.lib.units.derived.cos
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.derived.sin
import java.util.function.DoubleSupplier
import java.util.function.Supplier
import kotlin.math.PI
import kotlin.math.absoluteValue
import kotlin.math.max
import kotlin.math.min
import kotlin.math.pow
import kotlin.math.sqrt

class AutofaceReefCommand(
  private val drivetrain: Drive,
  val driver: DriverProfile,
  val driveX: DoubleSupplier,
  val driveY: DoubleSupplier,
  val turn: DoubleSupplier,
  val slowMode: () -> Boolean,
  private val gamePieceSupplier: Supplier<Constants.Universal.GamePiece>,
  private val forceStop: Supplier<Boolean>
) : Command() {
  private val thetaPID: PIDController<Radian, Velocity<Radian>>
  private var resetFlag: Boolean

  init {
    addRequirements(drivetrain)

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

    resetFlag = true
  }

  override fun initialize() {
    thetaPID.reset()
    resetFlag = true

    for ((i, rect) in DrivetrainConstants.BOUNDING_RECTANGLES.keys.withIndex()) {
      CustomLogger.recordOutput("Rectanangles/$i", rect)
    }
  }

  override fun execute() {
    CustomLogger.recordOutput("ActiveCommands/AutofaceReefCommand", true)

    val speed = driver.driveSpeedClampedSupplier(driveX, driveY, slowMode)
    val rotation = driver.rotationSpeedClampedSupplier(turn, slowMode)

    if (turn.asDouble.absoluteValue < DrivetrainConstants.TURN_ESCAPE_THRESHOLD &&
      gamePieceSupplier.get() == Constants.Universal.GamePiece.CORAL || !forceStop.get()
    ) {
      if (resetFlag) {
        thetaPID.reset()
        resetFlag = false
      }
      val setpointRotation = getWantedRotation(drivetrain.pose)

      val thetavel =
        thetaPID.calculate(drivetrain.rotation, setpointRotation) *
          if (RobotBase.isReal()) -1.0 else 1.0

      drivetrain.runSpeeds(
        ChassisSpeeds.fromFieldRelativeSpeeds(
          speed.first, speed.second, thetavel, drivetrain.pose.rotation
        )
      )
    } else {
      if (!resetFlag) resetFlag = true

      drivetrain.runSpeeds(
        ChassisSpeeds.fromFieldRelativeSpeeds(
          speed.first, speed.second, rotation, drivetrain.pose.rotation
        )
      )
    }
  }

  private fun getWantedRotation(curPose: Pose2d): Angle {
    val curTranslation = curPose.translation.translation2d

    // too close to autorotate, don't even try
    if (min(
        curTranslation.getDistance(DrivetrainConstants.reefCenterRed).absoluteValue,
        curTranslation.getDistance(DrivetrainConstants.reefCenterBlue).absoluteValue
      ) <
      (
        max(
            DrivetrainConstants.DRIVETRAIN_LENGTH.inMeters,
            DrivetrainConstants.DRIVETRAIN_WIDTH.inMeters
          )
          .meters * sqrt(2.0) / 2.0 +
          DrivetrainConstants.BUMPER_WIDTH +
          DrivetrainConstants.REEF_INRADIUS
        )
        .inMeters
    )
      return curPose.rotation

    // closest, second closest
    val twoClosestResults = MutableList(2) { Pair(Double.POSITIVE_INFINITY, 0.radians) }

    for ((rect, angle) in DrivetrainConstants.BOUNDING_RECTANGLES) {
      if (rect.contains(curTranslation)) return angle

      val dist = rect.getDistance(curTranslation)

      if (dist < twoClosestResults[0].first) {
        twoClosestResults[1] = twoClosestResults[0]
        twoClosestResults[0] = Pair(dist, angle)
      } else if (dist < twoClosestResults[1].first) {
        twoClosestResults[1] = Pair(dist, angle)
      }
    }

    // smoothing between closest two rects
    val (dist1, rot1) = twoClosestResults[0]
    val (dist2, rot2) = twoClosestResults[1]

    val t = dist1 / (dist1 + dist2 + 1e-9) // fraction toward rect1

    // raise to < 1 to make curver sharper
    val shapedT = t.pow(0.5)

    return Angle(
      (rot1.cos * (1 - shapedT) + rot2.cos * t), (rot1.sin * (1 - shapedT) + rot2.sin * t)
    )
  }

  override fun isFinished(): Boolean {
    return false
  }

  override fun end(interrupted: Boolean) {
    CustomLogger.recordOutput("ActiveCommands/AutofaceReefCommand", false)
  }
}
