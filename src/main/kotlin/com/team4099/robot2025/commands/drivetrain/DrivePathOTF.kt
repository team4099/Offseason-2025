package com.team4099.robot2025.commands.drivetrain

import com.pathplanner.lib.commands.FollowPathCommand
import com.pathplanner.lib.path.PathPlannerPath
import com.pathplanner.lib.path.Waypoint
import com.pathplanner.lib.util.DriveFeedforwards
import com.team4099.lib.logging.LoggedTunableValue
import com.team4099.robot2025.config.constants.Constants
import com.team4099.robot2025.config.constants.DrivetrainConstants
import com.team4099.robot2025.subsystems.drivetrain.drive.Drivetrain
import com.team4099.robot2025.util.AllianceFlipUtil
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.Command
import org.team4099.lib.geometry.Pose2d
import org.team4099.lib.geometry.Translation2d
import org.team4099.lib.pplib.PathPlannerHolonomicDriveController
import org.team4099.lib.pplib.PathPlannerHolonomicDriveController.Companion.GoalEndState
import org.team4099.lib.pplib.PathPlannerHolonomicDriveController.Companion.ModuleConfig
import org.team4099.lib.pplib.PathPlannerHolonomicDriveController.Companion.PathConstraints
import org.team4099.lib.pplib.PathPlannerHolonomicDriveController.Companion.RobotConfig
import org.team4099.lib.pplib.PathPlannerRotationPID
import org.team4099.lib.pplib.PathPlannerTranslationPID
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.derived.inMetersPerSecondPerMeter
import org.team4099.lib.units.derived.inMetersPerSecondPerMeterSeconds
import org.team4099.lib.units.derived.inMetersPerSecondPerMetersPerSecond
import org.team4099.lib.units.derived.inRadiansPerSecondPerRadian
import org.team4099.lib.units.derived.inRadiansPerSecondPerRadianPerSecond
import org.team4099.lib.units.derived.inRadiansPerSecondPerRadianSeconds
import org.team4099.lib.units.derived.metersPerSecondPerMetersPerSecond
import org.team4099.lib.units.derived.perMeter
import org.team4099.lib.units.derived.perMeterSeconds
import org.team4099.lib.units.derived.perRadian
import org.team4099.lib.units.derived.perRadianPerSecond
import org.team4099.lib.units.derived.perRadianSeconds
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.perSecond
import java.util.function.DoubleSupplier
import java.util.function.Supplier
import edu.wpi.first.math.geometry.Pose2d as WPIPose2d

/**
 * @property drivetrain Drivetrain.
 * @property driveX
 * @property driveY
 * @property turn
 * @property poseReferenceSupplier Supplier of current drivetrain pose.
 * @property poses List of poses for the path to go through. **WARNING: The rotation of each pose
 * should be the direction of travel, NOT the rotation of the swerve chassis. See
 * [PathPlanner documentation](https://pathplanner.dev/pplib-create-a-path-on-the-fly.html)**
 * @property goalEndState Goal end state for chassis.
 */
class DrivePathOTF(
  private val drivetrain: Drivetrain,
  private val driveX: DoubleSupplier,
  private val driveY: DoubleSupplier,
  private val turn: DoubleSupplier,
  private val poseReferenceSupplier: Supplier<WPIPose2d>,
  private val poses: List<Supplier<Pose2d>>,
  private val goalEndState: GoalEndState
) : Command() {
  private val DRIVE_ESCAPE_THRESHOLD = 0.4
  private val TURN_ESCAPE_THRESHOLD = 0.4

  private lateinit var command: FollowPathCommand

  private val thetakP =
    LoggedTunableValue(
      "Pathfollow/thetakP",
      Pair({ it.inRadiansPerSecondPerRadian }, { it.radians.perSecond.perRadian })
    )
  private val thetakI =
    LoggedTunableValue(
      "Pathfollow/thetakI",
      Pair(
        { it.inRadiansPerSecondPerRadianSeconds }, { it.radians.perSecond.perRadianSeconds }
      )
    )
  private val thetakD =
    LoggedTunableValue(
      "Pathfollow/thetakD",
      Pair(
        { it.inRadiansPerSecondPerRadianPerSecond },
        { it.radians.perSecond.perRadianPerSecond }
      )
    )

  private val poskP =
    LoggedTunableValue(
      "Pathfollow/posKP",
      DrivetrainConstants.PID.AUTO_POS_KP,
      Pair({ it.inMetersPerSecondPerMeter }, { it.meters.perSecond.perMeter })
    )
  private val poskI =
    LoggedTunableValue(
      "Pathfollow/posKI",
      DrivetrainConstants.PID.AUTO_POS_KI,
      Pair({ it.inMetersPerSecondPerMeterSeconds }, { it.meters.perSecond.perMeterSeconds })
    )
  private val poskD =
    LoggedTunableValue(
      "Pathfollow/posKD",
      DrivetrainConstants.PID.AUTO_POS_KD,
      Pair(
        { it.inMetersPerSecondPerMetersPerSecond }, { it.metersPerSecondPerMetersPerSecond }
      )
    )

  private val ppHolonomicDriveController: PathPlannerHolonomicDriveController

  init {
    addRequirements(drivetrain)

    if (RobotBase.isReal()) {
      thetakP.initDefault(DrivetrainConstants.PID.AUTO_THETA_PID_KP)
      thetakI.initDefault(DrivetrainConstants.PID.AUTO_THETA_PID_KI)
      thetakD.initDefault(DrivetrainConstants.PID.AUTO_THETA_PID_KD)
    } else {
      thetakP.initDefault(DrivetrainConstants.PID.SIM_AUTO_THETA_PID_KP)
      thetakI.initDefault(DrivetrainConstants.PID.SIM_AUTO_THETA_PID_KI)
      thetakD.initDefault(DrivetrainConstants.PID.SIM_AUTO_THETA_PID_KD)
    }

    ppHolonomicDriveController =
      PathPlannerHolonomicDriveController(
        PathPlannerTranslationPID(poskP.get(), poskI.get(), poskD.get()),
        PathPlannerRotationPID(thetakP.get(), thetakI.get(), thetakD.get()),
        Constants.Universal.LOOP_PERIOD_TIME
      )
  }

  override fun initialize() {
    val waypoints: List<Waypoint> =
      PathPlannerPath.waypointsFromPoses(
        listOf(poseReferenceSupplier.get()) + poses.map { pose2d -> pose2d.get().pose2d }
      )

    command =
      FollowPathCommand(
        PathPlannerPath(
          waypoints,
          PathConstraints(
            DrivetrainConstants.DRIVE_SETPOINT_MAX,
            DrivetrainConstants.MAX_AUTO_ACCEL,
            DrivetrainConstants.STEERING_VEL_MAX,
            DrivetrainConstants.STEERING_ACCEL_MAX
          )
            .pplibConstraints,
          null,
          goalEndState.pplibGoalEndState
        ),
        poseReferenceSupplier,
        { drivetrain.chassisState.chassisSpeedsWPILIB },
        { speeds: ChassisSpeeds, _: DriveFeedforwards -> drivetrain.setClosedLoop(speeds) },
        ppHolonomicDriveController.pplibController,
        RobotConfig(
          Constants.Universal.ROBOT_WEIGHT,
          Constants.Universal.ROBOT_MOI,
          ModuleConfig(
            DrivetrainConstants.WHEEL_DIAMETER / 2.0,
            DrivetrainConstants.DRIVE_SETPOINT_MAX,
            DrivetrainConstants.NITRILE_WHEEL_COF,
            DCMotor.getKrakenX60(1)
              .withReduction(1.0 / DrivetrainConstants.DRIVE_SENSOR_GEAR_RATIO),
            DrivetrainConstants.DRIVE_SUPPLY_CURRENT_LIMIT,
            1
          ),
          // fl, fr, bl, br
          Translation2d(
            DrivetrainConstants.DRIVETRAIN_LENGTH / 2.0 -
              DrivetrainConstants.WHEEL_DIAMETER / 2.0,
            -DrivetrainConstants.DRIVETRAIN_WIDTH / 2.0 +
              DrivetrainConstants.WHEEL_DIAMETER / 2.0
          ),
          Translation2d(
            DrivetrainConstants.DRIVETRAIN_LENGTH / 2.0 -
              DrivetrainConstants.WHEEL_DIAMETER / 2.0,
            DrivetrainConstants.DRIVETRAIN_WIDTH / 2.0 -
              DrivetrainConstants.WHEEL_DIAMETER / 2.0
          ),
          Translation2d(
            -DrivetrainConstants.DRIVETRAIN_LENGTH / 2.0 +
              DrivetrainConstants.WHEEL_DIAMETER / 2.0,
            -DrivetrainConstants.DRIVETRAIN_WIDTH / 2.0 +
              DrivetrainConstants.WHEEL_DIAMETER / 2.0
          ),
          Translation2d(
            -DrivetrainConstants.DRIVETRAIN_LENGTH / 2.0 +
              DrivetrainConstants.WHEEL_DIAMETER / 2.0,
            DrivetrainConstants.DRIVETRAIN_WIDTH / 2.0 -
              DrivetrainConstants.WHEEL_DIAMETER / 2.0
          )
        )
          .ppllibRobotConfig,
        { AllianceFlipUtil.shouldFlip() },
      )

    command.initialize()
  }

  override fun execute() {
    command.execute()
  }

  override fun isFinished(): Boolean {
    return command.isFinished ||
      driveX.asDouble >= DRIVE_ESCAPE_THRESHOLD ||
      driveY.asDouble >= DRIVE_ESCAPE_THRESHOLD ||
      turn.asDouble >= TURN_ESCAPE_THRESHOLD
  }

  override fun end(interrupted: Boolean) {
    // the end method of FollowPathCommand is very nice. they check if not interrupted and
    // goalendstate has velocity < 0.1 mps and with that, determines whether or not the drivetrain
    // should stop moving. tl;dr: we don't have to set drivetrain.closedloop again
    command.end(interrupted)
  }
}
