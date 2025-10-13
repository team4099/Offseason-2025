package com.team4099.robot2025.auto.mode

import choreo.Choreo
import choreo.trajectory.SwerveSample
import com.team4099.robot2025.commands.drivetrain.FollowChoreoPath
import com.team4099.robot2025.commands.drivetrain.ReefAlignCommand
import com.team4099.robot2025.config.ControlBoard
import com.team4099.robot2025.config.constants.ArmConstants
import com.team4099.robot2025.config.constants.Constants
import com.team4099.robot2025.subsystems.drivetrain.Drive
import com.team4099.robot2025.subsystems.elevator.Elevator
import com.team4099.robot2025.subsystems.superstructure.Superstructure
import com.team4099.robot2025.subsystems.vision.Vision
import com.team4099.robot2025.util.driver.Jessika
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.WaitCommand
import org.team4099.lib.geometry.Pose2d
import org.team4099.lib.smoothDeadband
import org.team4099.lib.units.base.inSeconds

class ThreeL4ProcessorLolipop(
  val drivetrain: Drive,
  val elevator: Elevator,
  val superstructure: Superstructure,
  val vision: Vision
) : SequentialCommandGroup() {
  init {
    addRequirements(drivetrain)

    addCommands(
      // ---------- 1: CENTER TO L4 ----------

      FollowChoreoPath(drivetrain, firstTrajectory),
      superstructure.prepScoreCoralCommand(Constants.Universal.CoralLevel.L4),
      WaitCommand(ArmConstants.TIME_TO_GOAL.inSeconds),
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
        ReefAlignCommand.BRANCH_ID.RIGHT
      ),
      WaitCommand(ArmConstants.TIME_TO_GOAL.inSeconds),

      // ---------- 2: L4 TO BOTTOM LOLLI' ----------

      ParallelCommandGroup(
        FollowChoreoPath(drivetrain, secondTrajectory), superstructure.intakeCoralCommand()
      ),

      // ---------- 3: BOTTOM LOLLI' TO L4 ----------

      FollowChoreoPath(drivetrain, thirdTrajectory),
      superstructure.prepScoreCoralCommand(Constants.Universal.CoralLevel.L4),
      WaitCommand(ArmConstants.TIME_TO_GOAL.inSeconds),
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
        ReefAlignCommand.BRANCH_ID.RIGHT
      ),
      WaitCommand(ArmConstants.TIME_TO_GOAL.inSeconds),

      // ---------- 4: L4 TO MIDDLE LOLLI' ----------

      ParallelCommandGroup(
        FollowChoreoPath(drivetrain, fourthTrajectory), superstructure.intakeCoralCommand()
      ),

      // ---------- 5: MIDDLE LOLLI' TO L4 ----------

      FollowChoreoPath(drivetrain, fifthTrajectory),
      superstructure.prepScoreCoralCommand(Constants.Universal.CoralLevel.L4),
      WaitCommand(ArmConstants.TIME_TO_GOAL.inSeconds),
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
        ReefAlignCommand.BRANCH_ID.LEFT
      ),
    )
  }

  companion object {
    private val firstTrajectory =
      Choreo.loadTrajectory<SwerveSample>("ThreeL4ProcessorLolipop/1CenterToL4.traj").get()
    private val secondTrajectory =
      Choreo.loadTrajectory<SwerveSample>("ThreeL4ProcessorLolipop/2L4ToBottom.traj").get()
    private val thirdTrajectory =
      Choreo.loadTrajectory<SwerveSample>("ThreeL4ProcessorLolipop/3BottomToL4.traj").get()
    private val fourthTrajectory =
      Choreo.loadTrajectory<SwerveSample>("ThreeL4ProcessorLolipop/4L4ToMiddle.traj").get()
    private val fifthTrajectory =
      Choreo.loadTrajectory<SwerveSample>("ThreeL4ProcessorLolipop/5MiddleToL4.traj").get()

    val startingPose = Pose2d(firstTrajectory.getInitialPose(false).get())
    val secondPose = Pose2d(secondTrajectory.getInitialPose(false).get())
    val thirdPose = Pose2d(thirdTrajectory.getInitialPose(false).get())
    val fourthPose = Pose2d(fourthTrajectory.getInitialPose(false).get())
    val fithPose = Pose2d(fifthTrajectory.getInitialPose(false).get())
  }
}
