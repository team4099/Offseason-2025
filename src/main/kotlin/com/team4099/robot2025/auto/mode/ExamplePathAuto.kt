package com.team4099.robot2025.auto.mode

import choreo.Choreo
import choreo.trajectory.SwerveSample
import com.team4099.robot2025.commands.drivetrain.FollowChoreoPath
import com.team4099.robot2025.subsystems.drivetrain.drive.Drivetrain
import com.team4099.robot2025.util.FMSData
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.WaitCommand
import org.team4099.lib.geometry.Pose2d

class ExamplePathAuto(val drivetrain: Drivetrain) : SequentialCommandGroup() {
  init {
    addRequirements(drivetrain)

    addCommands(
      WaitCommand(0.5),
      FollowChoreoPath(drivetrain, trajectory),
    )
  }

  companion object {
    val trajectory = Choreo.loadTrajectory<SwerveSample>("Example/ExamplePath.traj").get()
    val startingPose =
      Pose2d(trajectory.getInitialPose(FMSData.allianceColor == DriverStation.Alliance.Red).get())
  }
}
