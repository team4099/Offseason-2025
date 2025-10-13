package com.team4099.robot2025.auto.mode

import choreo.Choreo
import choreo.trajectory.SwerveSample
import com.team4099.robot2025.commands.drivetrain.FollowChoreoPath
import com.team4099.robot2025.commands.drivetrain.ReefAlignCommand
import com.team4099.robot2025.commands.drivetrain.TargetTagCommand
import com.team4099.robot2025.config.ControlBoard
import com.team4099.robot2025.config.constants.ArmConstants
import com.team4099.robot2025.config.constants.Constants
import com.team4099.robot2025.config.constants.RollersConstants
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
import org.team4099.lib.units.base.inches

class CenterL4Barge(
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

      // ---------- 2: L4 TO ALGAE REEF INTAKE ----------

      FollowChoreoPath(drivetrain, secondTrajectory),
      superstructure.intakeAlgaeCommand(Constants.Universal.AlgaeIntakeLevel.L2),
      WaitCommand(ArmConstants.TIME_TO_GOAL.inSeconds),
      TargetTagCommand(
        driver = Jessika(),
        { ControlBoard.forward.smoothDeadband(Constants.Joysticks.THROTTLE_DEADBAND) },
        { ControlBoard.strafe.smoothDeadband(Constants.Joysticks.THROTTLE_DEADBAND) },
        { ControlBoard.turn.smoothDeadband(Constants.Joysticks.TURN_DEADBAND) },
        { ControlBoard.slowMode },
        drivetrain,
        vision,
        0.inches
      ),

      // ---------- 3: REEF TO NET ----------

      ParallelCommandGroup(
        FollowChoreoPath(drivetrain, thirdTrajectory),
        WaitCommand(0.5)
          .andThen(
            superstructure.prepScoreAlgaeCommand(
              Constants.Universal.AlgaeScoringLevel.BARGE
            )
          )
      ),
      superstructure.scoreCommand(),
      WaitCommand(RollersConstants.GAMEPIECE_SPITOUT_THRESHOLD.inSeconds * 1.5),

      // ---------- 4: NET TO ALGAE REEF INTAKE ----------

      FollowChoreoPath(drivetrain, fourthTrajectory),
      superstructure.intakeAlgaeCommand(Constants.Universal.AlgaeIntakeLevel.L2),
      WaitCommand(ArmConstants.TIME_TO_GOAL.inSeconds),
      TargetTagCommand(
        driver = Jessika(),
        { ControlBoard.forward.smoothDeadband(Constants.Joysticks.THROTTLE_DEADBAND) },
        { ControlBoard.strafe.smoothDeadband(Constants.Joysticks.THROTTLE_DEADBAND) },
        { ControlBoard.turn.smoothDeadband(Constants.Joysticks.TURN_DEADBAND) },
        { ControlBoard.slowMode },
        drivetrain,
        vision,
        0.inches
      ),

      // ---------- 5: REEF TO NET ----------

      ParallelCommandGroup(
        FollowChoreoPath(drivetrain, fifthTrajectory),
        WaitCommand(0.5)
          .andThen(
            superstructure.prepScoreAlgaeCommand(
              Constants.Universal.AlgaeScoringLevel.BARGE
            )
          )
      ),
      superstructure.scoreCommand(),
    )
  }

  companion object {
    private val firstTrajectory =
      Choreo.loadTrajectory<SwerveSample>("CenterL4Barge/1StartToL4.traj").get()
    private val secondTrajectory =
      Choreo.loadTrajectory<SwerveSample>("CenterL4Barge/2L4ToReef.traj").get()
    private val thirdTrajectory =
      Choreo.loadTrajectory<SwerveSample>("CenterL4Barge/3ReefToNet.traj").get()
    private val fourthTrajectory =
      Choreo.loadTrajectory<SwerveSample>("CenterL4Barge/4NetToReef.traj").get()
    private val fifthTrajectory =
      Choreo.loadTrajectory<SwerveSample>("CenterL4Barge/5ReefToNet.traj").get()

    val startingPose = Pose2d(firstTrajectory.getInitialPose(false).get())
    private val secondPose = Pose2d(secondTrajectory.getInitialPose(false).get())
    private val thirdPose = Pose2d(thirdTrajectory.getInitialPose(false).get())
    private val fourthPose = Pose2d(fourthTrajectory.getInitialPose(false).get())
    private val fifthPose = Pose2d(fifthTrajectory.getInitialPose(false).get())
  }
}
