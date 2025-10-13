package com.team4099.robot2025.auto

import com.team4099.robot2025.auto.mode.CenterL4Barge
import com.team4099.robot2025.auto.mode.ExamplePathAuto
import com.team4099.robot2025.auto.mode.ThreeL4CoralStation
import com.team4099.robot2025.auto.mode.ThreeL4ProcessorLolipop
import com.team4099.robot2025.subsystems.drivetrain.Drive
import com.team4099.robot2025.subsystems.elevator.Elevator
import com.team4099.robot2025.subsystems.superstructure.Superstructure
import com.team4099.robot2025.subsystems.vision.Vision
import com.team4099.robot2025.util.AllianceFlipUtil
import edu.wpi.first.networktables.GenericEntry
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.WaitCommand
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser
import org.team4099.lib.units.base.Time
import org.team4099.lib.units.base.inSeconds
import org.team4099.lib.units.base.seconds

object AutonomousSelector {
  //  private var orientationChooser: SendableChooser<Angle> = SendableChooser()
  private var autonomousModeChooser: LoggedDashboardChooser<AutonomousMode> =
    LoggedDashboardChooser("AutonomousMode")
  private var waitBeforeCommandSlider: GenericEntry
  private var secondaryWaitInAuto: GenericEntry

  init {
    val autoTab = Shuffleboard.getTab("Pre-match")
    //    orientationChooser.setDefaultOption("Forward", 0.degrees)
    //    orientationChooser.addOption("Backwards", 180.degrees)
    //    orientationChooser.addOption("Left", 90.degrees)
    //    orientationChooser.addOption("Right", 270.degrees)
    //    autoTab.add("Starting Orientation", orientationChooser)

    autonomousModeChooser
      .addOption( // This is an example auto similarly to -1337 it is a placeholder so it should
        // not be used
        "Example Auto DO NOT RUN AT COMPETITION",
        AutonomousMode.EXAMPLE_AUTO
      )

    autonomousModeChooser.addOption("Center L4 + 2 Barge", AutonomousMode.CENTER_L4_BARGE)

    autonomousModeChooser.addOption(
      "Three L4 From Coral Station", AutonomousMode.THREE_L4_CORAL_STATION
    )

    autonomousModeChooser.addOption("Three L4 from Lollipops", AutonomousMode.THREE_L4_LOLLIPOP)

    autoTab.add("Mode", autonomousModeChooser.sendableChooser).withSize(4, 2).withPosition(2, 0)

    waitBeforeCommandSlider =
      autoTab
        .add("Wait Time Before Shooting", 0)
        .withSize(3, 2)
        .withPosition(0, 2)
        .withWidget(BuiltInWidgets.kTextView)
        .entry
    secondaryWaitInAuto =
      autoTab
        .add("Secondary Wait Time Between Shooting and Driving", 0)
        .withSize(3, 2)
        .withPosition(3, 2)
        .withWidget(BuiltInWidgets.kTextView)
        .entry
  }

  val waitTime: Time
    get() = waitBeforeCommandSlider.getDouble(0.0).seconds

  val secondaryWaitTime: Time
    get() = secondaryWaitInAuto.getDouble(0.0).seconds

  fun getCommand(
    drivetrain: Drive,
    elevator: Elevator,
    superstructure: Superstructure,
    vision: Vision
  ): Command {
    val mode = autonomousModeChooser.get()

    when (mode) {
      AutonomousMode.EXAMPLE_AUTO ->
        return WaitCommand(waitTime.inSeconds)
          .andThen({ drivetrain.pose = AllianceFlipUtil.apply(ExamplePathAuto.startingPose) })
          .andThen(ExamplePathAuto(drivetrain))
      AutonomousMode.CENTER_L4_BARGE ->
        return WaitCommand(waitTime.inSeconds)
          .andThen({ drivetrain.pose = AllianceFlipUtil.apply(CenterL4Barge.startingPose) })
          .andThen(CenterL4Barge(drivetrain, elevator, superstructure, vision))
      AutonomousMode.THREE_L4_CORAL_STATION ->
        return WaitCommand(waitTime.inSeconds)
          .andThen({
            drivetrain.pose = AllianceFlipUtil.apply(ThreeL4CoralStation.startingPose)
          })
          .andThen(ThreeL4CoralStation(drivetrain, elevator, superstructure, vision))
      AutonomousMode.THREE_L4_LOLLIPOP ->
        return WaitCommand(waitTime.inSeconds)
          .andThen({
            drivetrain.pose = AllianceFlipUtil.apply(ThreeL4ProcessorLolipop.startingPose)
          })
          .andThen(ThreeL4ProcessorLolipop(drivetrain, elevator, superstructure, vision))
      else -> return InstantCommand()
    }
  }

  fun getLoadingCommand(drivetrain: Drive): Command {
    return ExamplePathAuto(drivetrain)
  }

  private enum class AutonomousMode {
    // Delete this when real autos are made
    EXAMPLE_AUTO,
    CENTER_L4_BARGE,
    THREE_L4_CORAL_STATION,
    THREE_L4_LOLLIPOP
  }
}
