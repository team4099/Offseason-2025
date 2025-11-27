package com.team4099.robot2025.subsystems.superstructure.elevator

import com.team4099.lib.logging.LoggedTunableValue
import com.team4099.robot2025.config.constants.ElevatorConstants
import org.team4099.lib.units.base.inInches
import org.team4099.lib.units.base.inches

object ElevatorTunableValues {
  val minHeight =
    LoggedTunableValue(
      "Elevator/minPosition",
      ElevatorConstants.DOWNWARDS_EXTENSION_LIMIT,
      Pair({ it.inInches }, { it.inches })
    )

  val maxHeight =
    LoggedTunableValue(
      "Elevator/maxPosition",
      ElevatorConstants.UPWARDS_EXTENSION_LIMIT,
      Pair({ it.inInches }, { it.inches })
    )

  val idleHeight =
    LoggedTunableValue(
      "Elevator/idleHeight",
      ElevatorConstants.HEIGHTS.IDLE,
      Pair({ it.inInches }, { it.inches })
    )

  val idleCoralHeight =
    LoggedTunableValue(
      "Elevator/idleCoralHeight",
      ElevatorConstants.HEIGHTS.IDLE_CORAL,
      Pair({ it.inInches }, { it.inches })
    )

  val idleAlgaeHeight =
    LoggedTunableValue(
      "Elevator/idleAlgaeHeight",
      ElevatorConstants.HEIGHTS.IDLE_ALGAE,
      Pair({ it.inInches }, { it.inches })
    )

  val intakeCoralHeight =
    LoggedTunableValue(
      "Elevator/intakeCoralHeight",
      ElevatorConstants.HEIGHTS.INTAKE_CORAL,
      Pair({ it.inInches }, { it.inches })
    )

  val intakeAlgaeGroundHeight =
    LoggedTunableValue(
      "Elevator/intakeAlgaeGroundHeight",
      ElevatorConstants.HEIGHTS.INTAKE_ALGAE_GROUND,
      Pair({ it.inInches }, { it.inches })
    )

  val intakeAlgaeLowHeight =
    LoggedTunableValue(
      "Elevator/intakeAlgaeLowHeight",
      ElevatorConstants.HEIGHTS.INTAKE_ALGAE_LOW,
      Pair({ it.inInches }, { it.inches })
    )

  val intakeAlgaeHighHeight =
    LoggedTunableValue(
      "Elevator/intakeAlgaeHighHeight",
      ElevatorConstants.HEIGHTS.INTAKE_ALGAE_HIGH,
      Pair({ it.inInches }, { it.inches })
    )

  val L1Height =
    LoggedTunableValue(
      "Elevator/L1Height", ElevatorConstants.HEIGHTS.L1, Pair({ it.inInches }, { it.inches })
    )

  val L2Height =
    LoggedTunableValue(
      "Elevator/L2Height", ElevatorConstants.HEIGHTS.L2, Pair({ it.inInches }, { it.inches })
    )

  val L3Height =
    LoggedTunableValue(
      "Elevator/L3Height", ElevatorConstants.HEIGHTS.L3, Pair({ it.inInches }, { it.inches })
    )

  val L4Height =
    LoggedTunableValue(
      "Elevator/L4Height", ElevatorConstants.HEIGHTS.L4, Pair({ it.inInches }, { it.inches })
    )

  val processorHeight =
    LoggedTunableValue(
      "Elevator/processorHeight",
      ElevatorConstants.HEIGHTS.PROCESSOR,
      Pair({ it.inInches }, { it.inches })
    )

  val bargeHeight =
    LoggedTunableValue(
      "Elevator/bargeHeight",
      ElevatorConstants.HEIGHTS.BARGE,
      Pair({ it.inInches }, { it.inches })
    )

  val ejectHeight =
    LoggedTunableValue(
      "Elevator/ejectHeight",
      ElevatorConstants.HEIGHTS.EJECT,
      Pair({ it.inInches }, { it.inches })
    )

  val l1InitHeight =
    LoggedTunableValue(
      "Elevator/l1InitHeight",
      ElevatorConstants.HEIGHTS.L1_INIT,
      Pair({ it.inInches }, { it.inches })
    )
}
