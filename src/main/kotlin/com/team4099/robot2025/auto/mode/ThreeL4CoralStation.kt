package com.team4099.robot2025.auto.mode

import choreo.Choreo
import choreo.trajectory.SwerveSample
import com.team4099.robot2025.commands.drivetrain.FollowChoreoPath
import com.team4099.robot2025.commands.drivetrain.ReefAlignCommand
import com.team4099.robot2025.config.ControlBoard
import com.team4099.robot2025.config.constants.Constants
import com.team4099.robot2025.subsystems.drivetrain.drive.Drivetrain
import com.team4099.robot2025.subsystems.elevator.Elevator
import com.team4099.robot2025.subsystems.superstructure.Superstructure
import com.team4099.robot2025.subsystems.vision.Vision
import com.team4099.robot2025.util.AllianceFlipUtil
import com.team4099.robot2025.util.driver.Jessika
import edu.wpi.first.wpilibj2.command.Commands.runOnce
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import org.team4099.lib.geometry.Pose2d
import org.team4099.lib.smoothDeadband

class ThreeL4CoralStation(
  drivetrain: Drivetrain,
  elevator: Elevator,
  superstructure: Superstructure,
  vision: Vision
) : SequentialCommandGroup() {

  init {
    addRequirements(drivetrain)
    addCommands(
      // ---------- 1: CENTER TO L4 ----------

      FollowChoreoPath(drivetrain, trajectory1),
      ParallelCommandGroup(
        ReefAlignCommand(
          driver = Jessika(),
          { ControlBoard.forward.smoothDeadband(Constants.Joysticks.THROTTLE_DEADBAND) },
          { ControlBoard.strafe.smoothDeadband(Constants.Joysticks.THROTTLE_DEADBAND) },
          { ControlBoard.turn.smoothDeadband(Constants.Joysticks.TURN_DEADBAND) },
          { ControlBoard.slowMode },
          drivetrain,
          elevator,
          superstructure,
          vision,
          1
        ),
        superstructure.prepScoreCoralCommand(Constants.Universal.CoralLevel.L4)
      )
        .withTimeout(3.0),
      runOnce({
        drivetrain.resetFieldFrameEstimator(AllianceFlipUtil.apply(pose2))
      }), // reset pose after scoring

      // ---------- 2: L4 TO CORAL STATION ----------

      ParallelCommandGroup(
        FollowChoreoPath(drivetrain, trajectory2),
        superstructure.intakeCoralCommand()
      ).withTimeout(3.0),
      runOnce({ drivetrain.resetFieldFrameEstimator(AllianceFlipUtil.apply(pose3)) }),

      // ---------- 3: CORAL STATION TO L4 ----------

      FollowChoreoPath(drivetrain, trajectory3),
      ParallelCommandGroup(
        ReefAlignCommand(
          driver = Jessika(),
          { ControlBoard.forward.smoothDeadband(Constants.Joysticks.THROTTLE_DEADBAND) },
          { ControlBoard.strafe.smoothDeadband(Constants.Joysticks.THROTTLE_DEADBAND) },
          { ControlBoard.turn.smoothDeadband(Constants.Joysticks.TURN_DEADBAND) },
          { ControlBoard.slowMode },
          drivetrain,
          elevator,
          superstructure,
          vision,
          1
        ),
        superstructure.prepScoreCoralCommand(Constants.Universal.CoralLevel.L4)
      )
        .withTimeout(3.0),
      runOnce({ drivetrain.resetFieldFrameEstimator(AllianceFlipUtil.apply(pose4)) }),

      // ---------- 4: L4 TO CORAL STATION ----------

      ParallelCommandGroup(
        FollowChoreoPath(drivetrain, trajectory4),
        superstructure.intakeCoralCommand()
      ).withTimeout(3.0),
      runOnce({ drivetrain.resetFieldFrameEstimator(AllianceFlipUtil.apply(pose5)) }),

      // ---------- 5: CORAL STATION TO L4 ----------

      FollowChoreoPath(drivetrain, trajectory5),
      ParallelCommandGroup(
        ReefAlignCommand(
          driver = Jessika(),
          { ControlBoard.forward.smoothDeadband(Constants.Joysticks.THROTTLE_DEADBAND) },
          { ControlBoard.strafe.smoothDeadband(Constants.Joysticks.THROTTLE_DEADBAND) },
          { ControlBoard.turn.smoothDeadband(Constants.Joysticks.TURN_DEADBAND) },
          { ControlBoard.slowMode },
          drivetrain,
          elevator,
          superstructure,
          vision,
          1
        ),
        superstructure.prepScoreCoralCommand(Constants.Universal.CoralLevel.L4)
      )
        .withTimeout(3.0),
    )
  }

  companion object {
    private val trajectory1 =
      Choreo.loadTrajectory<SwerveSample>("ThreeL4CoralStation/1CenterToL4.traj").get()
    private val trajectory2 =
      Choreo.loadTrajectory<SwerveSample>("ThreeL4CoralStation/2L4ToCoralStation.traj").get()
    private val trajectory3 =
      Choreo.loadTrajectory<SwerveSample>("ThreeL4CoralStation/3CoralStationToL4.traj").get()
    private val trajectory4 =
      Choreo.loadTrajectory<SwerveSample>("ThreeL4CoralStation/4L4ToCoralStation.traj").get()
    private val trajectory5 =
      Choreo.loadTrajectory<SwerveSample>("ThreeL4CoralStation/5CoralStationToL4.traj").get()

    val startingPose = Pose2d(trajectory1.getInitialPose(false).get())
    val pose2 = Pose2d(trajectory2.getInitialPose(false).get())
    val pose3 = Pose2d(trajectory3.getInitialPose(false).get())
    val pose4 = Pose2d(trajectory4.getInitialPose(false).get())
    val pose5 = Pose2d(trajectory5.getInitialPose(false).get())
  }
}
