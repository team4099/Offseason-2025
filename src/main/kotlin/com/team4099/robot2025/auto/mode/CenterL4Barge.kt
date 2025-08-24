package com.team4099.robot2025.auto.mode

import choreo.Choreo
import choreo.trajectory.SwerveSample
import com.team4099.robot2025.commands.drivetrain.FollowChoreoPath
import com.team4099.robot2025.commands.drivetrain.ReefAlignCommand
import com.team4099.robot2025.commands.drivetrain.TargetTagCommand
import com.team4099.robot2025.config.ControlBoard
import com.team4099.robot2025.config.constants.Constants
import com.team4099.robot2025.config.constants.RollersConstants
import com.team4099.robot2025.subsystems.drivetrain.drive.Drivetrain
import com.team4099.robot2025.subsystems.elevator.Elevator
import com.team4099.robot2025.subsystems.superstructure.Superstructure
import com.team4099.robot2025.subsystems.vision.Vision
import com.team4099.robot2025.util.AllianceFlipUtil
import com.team4099.robot2025.util.driver.Jessika
import edu.wpi.first.wpilibj2.command.Commands.runOnce
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.WaitCommand
import org.team4099.lib.geometry.Pose2d
import org.team4099.lib.smoothDeadband
import org.team4099.lib.units.base.inSeconds
import org.team4099.lib.units.base.inches

class CenterL4Barge(
  val drivetrain: Drivetrain,
  val elevator: Elevator,
  val superstructure: Superstructure,
  val vision: Vision
) : SequentialCommandGroup() {
  init {
    addRequirements(drivetrain)

    addCommands(
      FollowChoreoPath(drivetrain, firstTrajectory),
      ParallelCommandGroup(
        superstructure.prepScoreCoralCommand(Constants.Universal.CoralLevel.L4),
        WaitCommand(1.5)
          .andThen(
            ReefAlignCommand(
              driver = Jessika(),
              {
                ControlBoard.forward.smoothDeadband(
                  Constants.Joysticks.THROTTLE_DEADBAND
                )
              },
              {
                ControlBoard.strafe.smoothDeadband(
                  Constants.Joysticks.THROTTLE_DEADBAND
                )
              },
              { ControlBoard.turn.smoothDeadband(Constants.Joysticks.TURN_DEADBAND) },
              { ControlBoard.slowMode },
              drivetrain,
              elevator,
              superstructure,
              vision,
              1
            )
          )
      )
        .withTimeout(3.0),
      runOnce({
        drivetrain.resetFieldFrameEstimator(AllianceFlipUtil.apply(secondPose))
      }), // reset pose after path
      FollowChoreoPath(drivetrain, secondTrajectory),
      ParallelCommandGroup(
        superstructure.intakeAlgaeCommand(Constants.Universal.AlgaeIntakeLevel.L2),
        WaitCommand(1.5)
          .andThen(
            TargetTagCommand(
              driver = Jessika(),
              {
                ControlBoard.forward.smoothDeadband(
                  Constants.Joysticks.THROTTLE_DEADBAND
                )
              },
              {
                ControlBoard.strafe.smoothDeadband(
                  Constants.Joysticks.THROTTLE_DEADBAND
                )
              },
              { ControlBoard.turn.smoothDeadband(Constants.Joysticks.TURN_DEADBAND) },
              { ControlBoard.slowMode },
              drivetrain,
              vision,
              0.inches
            )
          )
      )
        .withTimeout(3.0),
      runOnce({
        drivetrain.resetFieldFrameEstimator(AllianceFlipUtil.apply(thirdPose))
      }), // reset pose after path
      ParallelCommandGroup(
        FollowChoreoPath(drivetrain, thirdTrajectory),
        WaitCommand(0.5)
          .andThen(
            superstructure.prepScoreAlgaeCommand(
              Constants.Universal.AlgaeScoringLevel.BARGE
            )
          )
      ),
      superstructure
        .scoreCommand()
        .withTimeout(RollersConstants.GAMEPIECE_SPITOUT_THRESHOLD.inSeconds * 1.5),
      runOnce({
        drivetrain.resetFieldFrameEstimator(AllianceFlipUtil.apply(fourthPose))
      }), // reset pose after path
      FollowChoreoPath(drivetrain, fourthTrajectory),
      ParallelCommandGroup(
        superstructure.intakeAlgaeCommand(Constants.Universal.AlgaeIntakeLevel.L2),
        WaitCommand(1.5)
          .andThen(
            TargetTagCommand(
              driver = Jessika(),
              {
                ControlBoard.forward.smoothDeadband(
                  Constants.Joysticks.THROTTLE_DEADBAND
                )
              },
              {
                ControlBoard.strafe.smoothDeadband(
                  Constants.Joysticks.THROTTLE_DEADBAND
                )
              },
              { ControlBoard.turn.smoothDeadband(Constants.Joysticks.TURN_DEADBAND) },
              { ControlBoard.slowMode },
              drivetrain,
              vision,
              0.inches
            )
          )
      )
        .withTimeout(3.0),
      runOnce({
        drivetrain.resetFieldFrameEstimator(AllianceFlipUtil.apply(fifthPose))
      }), // reset pose after path
      ParallelCommandGroup(
        FollowChoreoPath(drivetrain, fifthTrajectory),
        WaitCommand(0.5)
          .andThen(
            superstructure.prepScoreAlgaeCommand(
              Constants.Universal.AlgaeScoringLevel.BARGE
            )
          )
      ),
      superstructure.scoreCommand()
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
