package com.team4099.robot2025.subsystems.canRange

import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs
import org.team4099.lib.units.base.inMeters
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.volts

interface CANRangeIO {
    class CANRangeIOInputs : LoggableInputs{
        var distance = 0.0.meters
        var isDetected = false
        var supplyVoltage = 0.0.volts
        var ambientSignal =0.0
        override fun toLog(table: LogTable) {
            table.put("distance", distance.inMeters)
            table.put("isDetected", isDetected)
            table.put("supplyVoltage", supplyVoltage.inVolts)
            table.put("ambientSignal", ambientSignal)
        }



        override fun fromLog(table: LogTable) {
            table.get("distance", distance.inMeters).let {distance = it.meters}
            table.get("supplyVoltage", supplyVoltage.inVolts).let {supplyVoltage = it.volts}
            table.get("isDetected", isDetected)
            table.get("ambientSignal", ambientSignal).let {ambientSignal = it}
        }

    }
    fun updateInputs(inputs: CANRangeIO.CANRangeIOInputs){
    }
}