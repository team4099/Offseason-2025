package com.team4099.robot2025.subsystems.canRange

import com.ctre.phoenix6.StatusSignal
import com.ctre.phoenix6.configs.CANrangeConfiguration
import com.ctre.phoenix6.hardware.core.CoreCANrange
import com.team4099.robot2025.config.constants.Constants
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.Voltage

import org.team4099.lib.units.base.meters
import org.team4099.lib.units.derived.volts

object IOCANRange: CANRangeIO {
    private val CANrange: CoreCANrange = CoreCANrange(Constants.CanRange.CANRANGEID)
    private val config: CANrangeConfiguration = CANrangeConfiguration()

    var AmbientSignal: StatusSignal<Double>
    var distance: StatusSignal<Distance>
    var isDetected: StatusSignal<Boolean>
    var supplyVoltageSignal: StatusSignal<Voltage>


    init {
        CANrange.clearStickyFaults()
        config.ProximityParams.MinSignalStrengthForValidMeasurement = 100.0
        config.ProximityParams.ProximityThreshold = 2500.0

        AmbientSignal = CANrange.ambientSignal
        distance = CANrange.distance
        isDetected = CANrange.isDetected
        supplyVoltageSignal = CANrange.supplyVoltage
    }

    override fun updateInputs(inputs: CANRangeIO.CANRangeIOInputs){
        inputs.supplyVoltage = supplyVoltageSignal.valueAsDouble.volts
        inputs.isDetected = isDetected.value
        inputs.ambientSignal = AmbientSignal.valueAsDouble
        inputs.distance = distance.valueAsDouble.meters
    }
}