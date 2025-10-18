package com.team4099.robot2025.auto.mode

import choreo.Choreo
import choreo.trajectory.SwerveSample
import com.team4099.robot2025.RobotContainer.superstructure
import com.team4099.robot2025.commands.drivetrain.CoolerTargetTagCommand
import com.team4099.robot2025.commands.drivetrain.FollowChoreoPath
import com.team4099.robot2025.config.constants.ArmConstants
import com.team4099.robot2025.config.constants.Constants
import com.team4099.robot2025.subsystems.drivetrain.Drive
import com.team4099.robot2025.subsystems.vision.Vision
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.WaitCommand
import org.team4099.lib.geometry.Pose2d
import org.team4099.lib.units.base.inSeconds

class CenterScore (val drivetrain: Drive,val vision: Vision) : SequentialCommandGroup() {
    init {
      addRequirements(drivetrain)

      addCommands(
        WaitCommand(0.5),
        FollowChoreoPath(drivetrain, firstTrajectory),
        superstructure.prepScoreCoralCommand(Constants.Universal.CoralLevel.L4),
        WaitCommand(ArmConstants.TIME_TO_GOAL.inSeconds),
        CoolerTargetTagCommand.alignLeftCommand(drivetrain, vision).withTimeout(3.0),
        WaitCommand(.5),
        superstructure.scoreCommand()
      )
    }

    companion object {
      val firstTrajectory =
        Choreo.loadTrajectory<SwerveSample>("CenterScore/1CenterToL4.traj").get()

      val startingPose = Pose2d(firstTrajectory.getInitialPose(false).get())
    }
  }
