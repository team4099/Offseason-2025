package com.team4099.robot2025.commands.drivetrain

import choreo.trajectory.SwerveSample
import choreo.trajectory.Trajectory
import com.team4099.lib.logging.LoggedTunableValue
import com.team4099.lib.math.asPose2d
import com.team4099.lib.math.asTransform2d
import com.team4099.lib.trajectory.CustomHolonomicDriveController
import com.team4099.robot2025.config.constants.DrivetrainConstants
import com.team4099.robot2025.subsystems.drivetrain.drive.Drivetrain
import com.team4099.robot2025.subsystems.superstructure.Request.DrivetrainRequest
import com.team4099.robot2025.util.CustomLogger
import com.team4099.robot2025.util.FMSData
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.Command
import org.team4099.lib.controller.PIDController
import org.team4099.lib.geometry.Pose2d
import org.team4099.lib.hal.Clock
import org.team4099.lib.units.Velocity
import org.team4099.lib.units.base.Meter
import org.team4099.lib.units.base.inSeconds
import org.team4099.lib.units.base.inches
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.Radian
import org.team4099.lib.units.derived.degrees
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
import kotlin.math.PI

class FollowChoreoPath(val drivetrain: Drivetrain, val trajectory: Trajectory<SwerveSample>) :
  Command() {

  private val xPID: PIDController<Meter, Velocity<Meter>>
  private val yPID: PIDController<Meter, Velocity<Meter>>
  private val thetaPID: PIDController<Radian, Velocity<Radian>>

  private var trajCurTime = 0.0.seconds
  private var trajStartTime = 0.0.seconds

  val thetakP =
    LoggedTunableValue(
      "Pathfollow/thetakP",
      Pair({ it.inRadiansPerSecondPerRadian }, { it.radians.perSecond.perRadian })
    )
  val thetakI =
    LoggedTunableValue(
      "Pathfollow/thetakI",
      Pair(
        { it.inRadiansPerSecondPerRadianSeconds }, { it.radians.perSecond.perRadianSeconds }
      )
    )
  val thetakD =
    LoggedTunableValue(
      "Pathfollow/thetakD",
      Pair(
        { it.inRadiansPerSecondPerRadianPerSecond },
        { it.radians.perSecond.perRadianPerSecond }
      )
    )

  val poskP =
    LoggedTunableValue(
      "Pathfollow/posKP",
      DrivetrainConstants.PID.AUTO_POS_KP,
      Pair({ it.inMetersPerSecondPerMeter }, { it.meters.perSecond.perMeter })
    )
  val poskI =
    LoggedTunableValue(
      "Pathfollow/posKI",
      DrivetrainConstants.PID.AUTO_POS_KI,
      Pair({ it.inMetersPerSecondPerMeterSeconds }, { it.meters.perSecond.perMeterSeconds })
    )
  val poskD =
    LoggedTunableValue(
      "Pathfollow/posKD",
      DrivetrainConstants.PID.AUTO_POS_KD,
      Pair(
        { it.inMetersPerSecondPerMetersPerSecond }, { it.metersPerSecondPerMetersPerSecond }
      )
    )

  val swerveDriveController: CustomHolonomicDriveController

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

    xPID = PIDController(poskP.get(), poskI.get(), poskD.get())
    yPID = PIDController(poskP.get(), poskI.get(), poskD.get())
    thetaPID = PIDController(thetakP.get(), thetakI.get(), thetakD.get())

    thetaPID.enableContinuousInput(-PI.radians, PI.radians)

    swerveDriveController =
      CustomHolonomicDriveController(
        xPID.wpiPidController, yPID.wpiPidController, thetaPID.wpiPidController
      )

    swerveDriveController.setTolerance(Pose2d(3.inches, 3.inches, 1.5.degrees).pose2d)
  }

  override fun initialize() {
    trajStartTime = Clock.fpgaTime
    thetaPID.reset()
    xPID.reset()
    yPID.reset()
  }

  override fun execute() {
    trajCurTime = Clock.fpgaTime - trajStartTime

    val desiredState =
      trajectory
        .sampleAt(trajCurTime.inSeconds, FMSData.allianceColor == DriverStation.Alliance.Red)
        .get()

    val poseReference =
      drivetrain
        .odomTField
        .inverse()
        .asPose2d()
        .transformBy(drivetrain.fieldTRobot.asTransform2d())
        .pose2d

    drivetrain.targetPose = Pose2d(desiredState.pose)

    val nextDriveState = swerveDriveController.calculate(poseReference, desiredState)
    drivetrain.currentRequest = DrivetrainRequest.ClosedLoop(nextDriveState)

    if (thetakP.hasChanged()) thetaPID.proportionalGain = thetakP.get()
    if (thetakI.hasChanged()) thetaPID.integralGain = thetakI.get()
    if (thetakD.hasChanged()) thetaPID.derivativeGain = thetakD.get()

    if (poskP.hasChanged()) {
      xPID.proportionalGain = poskP.get()
      yPID.proportionalGain = poskP.get()
    }
    if (poskI.hasChanged()) {
      xPID.integralGain = poskI.get()
      yPID.integralGain = poskI.get()
    }
    if (poskD.hasChanged() && poskD.hasChanged()) {
      xPID.derivativeGain = poskD.get()
      yPID.derivativeGain = poskD.get()
    }
  }

  override fun isFinished(): Boolean {
    trajCurTime = Clock.fpgaTime - trajStartTime
    return trajCurTime > trajectory.totalTime.seconds
  }

  override fun end(interrupted: Boolean) {
    CustomLogger.recordDebugOutput("ActiveCommands/FollowChoreoPath", false)
    drivetrain.currentRequest =
      DrivetrainRequest.OpenLoop(
        0.0.radians.perSecond, Pair(0.0.meters.perSecond, 0.0.meters.perSecond)
      )
  }
}
