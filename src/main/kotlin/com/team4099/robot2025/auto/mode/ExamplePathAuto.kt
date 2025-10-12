package com.team4099.robot2025.auto.mode

import choreo.Choreo
import choreo.trajectory.SwerveSample
import com.team4099.robot2025.commands.drivetrain.FollowChoreoPath
import com.team4099.robot2025.subsystems.drivetrain.CommandSwerveDrive
import com.team4099.robot2025.subsystems.drivetrain.Drive
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.WaitCommand
import org.team4099.lib.geometry.Pose2d

class ExamplePathAuto(val drivetrain: Drive) : SequentialCommandGroup() {
  init {
    addRequirements(drivetrain)

    addCommands(
      WaitCommand(0.5),
      FollowChoreoPath(drivetrain, trajectory),
    )
  }

  companion object {
    val trajectory = Choreo.loadTrajectory<SwerveSample>("Example/ExamplePath.traj").get()

    // don't flip pose: poses are robot relative since field frame estimator was reset
    val startingPose = Pose2d(trajectory.getInitialPose(false).get())
  }
}
