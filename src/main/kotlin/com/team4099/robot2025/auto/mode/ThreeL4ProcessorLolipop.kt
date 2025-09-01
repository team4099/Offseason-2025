package com.team4099.robot2025.auto.mode

import choreo.Choreo
import choreo.trajectory.SwerveSample
import com.team4099.robot2025.commands.drivetrain.FollowChoreoPath
import com.team4099.robot2025.commands.drivetrain.ReefAlignCommand
import com.team4099.robot2025.config.ControlBoard
import com.team4099.robot2025.config.constants.ArmConstants
import com.team4099.robot2025.config.constants.Constants
import com.team4099.robot2025.subsystems.drivetrain.drive.Drivetrain
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
  val drivetrain: Drivetrain,
  val elevator: Elevator,
  val superstructure: Superstructure,
  val vision: Vision
) : SequentialCommandGroup() {
  init {
    addRequirements(drivetrain)
    addCommands(
      WaitCommand(.1),
      FollowChoreoPath(drivetrain, firstTrajectory),
      ParallelCommandGroup(
        WaitCommand(ArmConstants.TIME_TO_GOAL.inSeconds)
          .andThen(
            ReefAlignCommand(
              driver = Jessika(),
              {
                ControlBoard.forward.smoothDeadband(Constants.Joysticks.THROTTLE_DEADBAND)
              },
              {
                ControlBoard.strafe.smoothDeadband(Constants.Joysticks.THROTTLE_DEADBAND)
              },
              { ControlBoard.turn.smoothDeadband(Constants.Joysticks.TURN_DEADBAND) },
              { ControlBoard.slowMode },
              drivetrain,
              elevator,
              superstructure,
              vision,
              ReefAlignCommand.BRANCH_ID.RIGHT
            )
          ),
        superstructure.prepScoreCoralCommand(Constants.Universal.CoralLevel.L4)
      ),
      ParallelCommandGroup(
        FollowChoreoPath(drivetrain, secondTrajectory), superstructure.intakeCoralCommand()
      ),
      FollowChoreoPath(drivetrain, thirdTrajectory),
      ParallelCommandGroup(
        WaitCommand(ArmConstants.TIME_TO_GOAL.inSeconds)
          .andThen(
            ReefAlignCommand(
              driver = Jessika(),
              {
                ControlBoard.forward.smoothDeadband(Constants.Joysticks.THROTTLE_DEADBAND)
              },
              {
                ControlBoard.strafe.smoothDeadband(Constants.Joysticks.THROTTLE_DEADBAND)
              },
              { ControlBoard.turn.smoothDeadband(Constants.Joysticks.TURN_DEADBAND) },
              { ControlBoard.slowMode },
              drivetrain,
              elevator,
              superstructure,
              vision,
              ReefAlignCommand.BRANCH_ID.RIGHT
            )
          ),
        superstructure.prepScoreCoralCommand(Constants.Universal.CoralLevel.L4)
      ),
      ParallelCommandGroup(
        FollowChoreoPath(drivetrain, fourthTrajectory), superstructure.intakeCoralCommand()
      ),
      FollowChoreoPath(drivetrain, fifthTrajectory),
      ParallelCommandGroup(
        WaitCommand(ArmConstants.TIME_TO_GOAL.inSeconds)
          .andThen(
            ReefAlignCommand(
              driver = Jessika(),
              {
                ControlBoard.forward.smoothDeadband(Constants.Joysticks.THROTTLE_DEADBAND)
              },
              {
                ControlBoard.strafe.smoothDeadband(Constants.Joysticks.THROTTLE_DEADBAND)
              },
              { ControlBoard.turn.smoothDeadband(Constants.Joysticks.TURN_DEADBAND) },
              { ControlBoard.slowMode },
              drivetrain,
              elevator,
              superstructure,
              vision,
              ReefAlignCommand.BRANCH_ID.RIGHT
            )
          ),
        superstructure.prepScoreCoralCommand(Constants.Universal.CoralLevel.L4)
      ),
    )
  }

  companion object {
    private val firstTrajectory =
      Choreo.loadTrajectory<SwerveSample>("/ThreeL4ProcessorLolipop/First Path").get()
    private val secondTrajectory =
      Choreo.loadTrajectory<SwerveSample>("/ThreeL4ProcessorLolipop/Second Path").get()
    private val thirdTrajectory =
      Choreo.loadTrajectory<SwerveSample>("/ThreeL4ProcessorLolipop/Third Path").get()
    private val fourthTrajectory =
      Choreo.loadTrajectory<SwerveSample>("/ThreeL4ProcessorLolipop/Fourth Path").get()
    private val fifthTrajectory =
      Choreo.loadTrajectory<SwerveSample>("/ThreeL4ProcessorLolipop/Fifth Path").get()

    val startingPose = Pose2d(firstTrajectory.getInitialPose(false).get())
    val secondPose = Pose2d(secondTrajectory.getInitialPose(false).get())
    val thirdPose = Pose2d(thirdTrajectory.getInitialPose(false).get())
    val fourthPose = Pose2d(fourthTrajectory.getInitialPose(false).get())
    val fithPose = Pose2d(fifthTrajectory.getInitialPose(false).get())
  }
}
