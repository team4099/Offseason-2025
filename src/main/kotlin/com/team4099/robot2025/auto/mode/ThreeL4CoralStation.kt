package com.team4099.robot2025.auto.mode

import choreo.Choreo
import choreo.trajectory.SwerveSample
import com.team4099.robot2025.commands.drivetrain.CoolerTargetTagCommand
import com.team4099.robot2025.commands.drivetrain.FollowChoreoPath
import com.team4099.robot2025.config.constants.ArmConstants
import com.team4099.robot2025.config.constants.Constants
import com.team4099.robot2025.subsystems.drivetrain.Drive
import com.team4099.robot2025.subsystems.elevator.Elevator
import com.team4099.robot2025.subsystems.superstructure.Superstructure
import com.team4099.robot2025.subsystems.vision.Vision
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.WaitCommand
import org.team4099.lib.geometry.Pose2d
import org.team4099.lib.units.base.inSeconds

class ThreeL4CoralStation(
  val drivetrain: Drive,
  val elevator: Elevator,
  val superstructure: Superstructure,
  val vision: Vision
) : SequentialCommandGroup() {

  init {
    addRequirements(drivetrain)
    addCommands(
      // ---------- 1: CENTER TO L4 ----------

      FollowChoreoPath(drivetrain, trajectory1),
      superstructure.prepScoreCoralCommand(Constants.Universal.CoralLevel.L3),
      WaitCommand(ArmConstants.TIME_TO_GOAL.inSeconds),
      CoolerTargetTagCommand.alignLeftCommand(drivetrain, vision),
      superstructure.scoreCommand(),
      WaitCommand(ArmConstants.TIME_TO_GOAL.inSeconds),

      // ---------- 2: L4 TO CORAL STATION ----------

      ParallelCommandGroup(
        FollowChoreoPath(drivetrain, trajectory2), superstructure.intakeCoralCommand()
      ),

      // ---------- 3: CORAL STATION TO L4 ----------

      FollowChoreoPath(drivetrain, trajectory3),
      superstructure.prepScoreCoralCommand(Constants.Universal.CoralLevel.L4),
      WaitCommand(ArmConstants.TIME_TO_GOAL.inSeconds),
      CoolerTargetTagCommand.alignRightCommand(drivetrain, vision),
      superstructure.scoreCommand(),
      WaitCommand(ArmConstants.TIME_TO_GOAL.inSeconds),

      // ---------- 4: L4 TO CORAL STATION ----------

      ParallelCommandGroup(
        FollowChoreoPath(drivetrain, trajectory4), superstructure.intakeCoralCommand()
      ),

      // ---------- 5: CORAL STATION TO L4 ----------

      FollowChoreoPath(drivetrain, trajectory5),
      superstructure.prepScoreCoralCommand(Constants.Universal.CoralLevel.L4),
      WaitCommand(ArmConstants.TIME_TO_GOAL.inSeconds),
      CoolerTargetTagCommand.alignLeftCommand(drivetrain, vision),
      superstructure.scoreCommand(),
      WaitCommand(ArmConstants.TIME_TO_GOAL.inSeconds),
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
