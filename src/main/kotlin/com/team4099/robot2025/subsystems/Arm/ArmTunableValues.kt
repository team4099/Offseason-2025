package com.team4099.robot2025.subsystems.Arm

import com.team4099.lib.logging.LoggedTunableValue
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inDegrees
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.inVoltsPerDegree
import org.team4099.lib.units.derived.inVoltsPerDegreePerSecond
import org.team4099.lib.units.derived.inVoltsPerDegreeSeconds
import org.team4099.lib.units.derived.inVoltsPerRadianPerSecond
import org.team4099.lib.units.derived.inVoltsPerRadianPerSecondPerSecond
import org.team4099.lib.units.derived.perDegree
import org.team4099.lib.units.derived.perDegreePerSecond
import org.team4099.lib.units.derived.perDegreeSeconds
import org.team4099.lib.units.derived.perRadianPerSecond
import org.team4099.lib.units.derived.perRadianPerSecondPerSecond
import org.team4099.lib.units.derived.volts

object ArmTunableValues {

  val armkS = LoggedTunableValue("arm/kS", Pair({ it.inVolts }, { it.volts }))
  val armkV =
    LoggedTunableValue(
      "arm/kV", Pair({ it.inVoltsPerRadianPerSecond }, { it.volts.perRadianPerSecond })
    )

  val armkA =
    LoggedTunableValue(
      "arm/kA",
      Pair({ it.inVoltsPerRadianPerSecondPerSecond }, { it.volts.perRadianPerSecondPerSecond })
    )
  val armkG = LoggedTunableValue("arm/kG", Pair({ it.inVolts }, { it.volts }))

  val armkP = LoggedTunableValue("arm/kP", Pair({ it.inVoltsPerDegree }, { it.volts.perDegree }))
  val armkI =
    LoggedTunableValue(
      "arm/kI", Pair({ it.inVoltsPerDegreeSeconds }, { it.volts.perDegreeSeconds })
    )
  val armkD =
    LoggedTunableValue(
      "arm/kD", Pair({ it.inVoltsPerDegreePerSecond }, { it.volts.perDegreePerSecond })
    )

  object Angles {
    val idleAngle = LoggedTunableValue("arm/idleAngle", Pair({ it.inDegrees }, { it.degrees }))

    val l1PrepAngle = LoggedTunableValue("arm/l1PrepAngle", Pair({ it.inDegrees }, { it.degrees }))
    val l2PrepAngle = LoggedTunableValue("arm/l2PrepAngle", Pair({ it.inDegrees }, { it.degrees }))
    val l3PrepAngle = LoggedTunableValue("arm/l3PrepAngle", Pair({ it.inDegrees }, { it.degrees }))
    val l4PrepAngle = LoggedTunableValue("arm/l4PrepAngle", Pair({ it.inDegrees }, { it.degrees }))

    val bargeAngle = LoggedTunableValue("arm/bargeAngle", Pair({ it.inDegrees }, { it.degrees }))
    val processorAngle =
      LoggedTunableValue("arm/processorAngle", Pair({ it.inDegrees }, { it.degrees }))
  }
}
