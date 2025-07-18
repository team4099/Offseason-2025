package com.team4099.robot2025.subsystems.Arm

import com.team4099.lib.logging.LoggedTunableValue
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.team4099.lib.controller.ArmFeedforward
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

class Arm(val io:ArmIO) : SubsystemBase() {
    val inputs = ArmIO.ArmIOInputs()

    private val armkS = LoggedTunableValue("arm/kS", Pair({ it.inVolts }, { it.volts }))
    private val armkV =
        LoggedTunableValue(
            "arm/kV", Pair({ it.inVoltsPerRadianPerSecond }, { it.volts.perRadianPerSecond })
        )
    private val armkA =
        LoggedTunableValue(
            "arm/kA",
            Pair({ it.inVoltsPerRadianPerSecondPerSecond }, { it.volts.perRadianPerSecondPerSecond })
        )
    private val armkG = LoggedTunableValue("arm/kG", Pair({ it.inVolts }, { it.volts }))

    var armFeedForward: ArmFeedforward

    private val armkP =
        LoggedTunableValue("arm/kP", Pair({ it.inVoltsPerDegree }, { it.volts.perDegree }))
    private val armkI =
        LoggedTunableValue(
            "arm/kI", Pair({ it.inVoltsPerDegreeSeconds }, { it.volts.perDegreeSeconds })
        )
    private val armkD =
        LoggedTunableValue(
            "arm/kD", Pair({ it.inVoltsPerDegreePerSecond }, { it.volts.perDegreePerSecond })
        )



    override fun periodic(){
        io.updateInputs(inputs)
        if (Ar)
    }


    companion object{
        enum class ArmStates{
            UNINIT,
            HOME,
            OPEN_LOOP,
            TARGETING_POS
        }
    }
}