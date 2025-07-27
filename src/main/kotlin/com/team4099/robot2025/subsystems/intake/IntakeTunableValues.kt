package com.team4099.robot2025.subsystems.Intake

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

object IntakeTunableValues {
    val coralPosition = LoggedTunableValue("intake/coralPosition", Pair({ it.inDegrees }, { it.degrees }))
    val coralRollerVoltage = LoggedTunableValue("intake/coralRollerVoltage", Pair({ it.inVolts }, { it.volts }))
}
