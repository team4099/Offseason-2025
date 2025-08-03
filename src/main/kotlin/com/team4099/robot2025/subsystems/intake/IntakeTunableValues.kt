package com.team4099.robot2025.subsystems.intake

import com.team4099.lib.logging.LoggedTunableValue
import com.team4099.robot2025.Robot
import com.team4099.robot2025.config.constants.IntakeConstants
import org.team4099.lib.units.derived.Radian
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inDegrees
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.volts

object IntakeTunableValues {
  val stowPosition =
    LoggedTunableValue(
      "intake/coralPosition",
      IntakeConstants.ANGLES.STOW_ANGLE,
      Pair({ it.inDegrees }, { it.degrees })
    )
  val idleRollerVoltage =
    LoggedTunableValue(
      "intake/coralRollerVoltage",
      IntakeConstants.Rollers.IDLE_VOLTAGE,
      Pair({ it.inVolts }, { it.volts })
    )

  val coralPosition =
    LoggedTunableValue(
      "intake/coralPosition",
      IntakeConstants.ANGLES.INTAKE_ANGLE,
      Pair({ it.inDegrees }, { it.degrees })
    )
  val coralRollerVoltage =
    LoggedTunableValue(
      "intake/coralRollerVoltage",
      IntakeConstants.Rollers.INTAKE_VOLTAGE,
      Pair({ it.inVolts }, { it.volts })
    )

  val idlePosition: LoggedTunableValue<Radian>
    get() = if (Robot.isAutonomous) coralPosition else stowPosition
}
