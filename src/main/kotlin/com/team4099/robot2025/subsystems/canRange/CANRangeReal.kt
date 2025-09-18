package com.team4099.robot2025.subsystems.canRange

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.StatusSignal
import com.ctre.phoenix6.configs.CANrangeConfiguration
import com.ctre.phoenix6.hardware.core.CoreCANrange
import com.team4099.robot2025.config.constants.Constants
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.Voltage
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.derived.volts

object CANRangeReal : CANRangeIO {
  private val canRange: CoreCANrange = CoreCANrange(Constants.CanRange.CANRANGE_ID)
  private val config: CANrangeConfiguration = CANrangeConfiguration()

  private var ambientSignal: StatusSignal<Double>
  private var distance: StatusSignal<Distance>
  private var isDetected: StatusSignal<Boolean>
  private var supplyVoltageSignal: StatusSignal<Voltage>

  init {
    canRange.clearStickyFaults()
    config.ProximityParams.MinSignalStrengthForValidMeasurement = 100.0
    config.ProximityParams.ProximityThreshold = .15

    ambientSignal = canRange.ambientSignal
    // NOTE(Aryan): distance signal doesn't need to refresh in updateInputs if you set getDistance
    // parameter to true (KEEP THIS LINE!!!)
    distance = canRange.getDistance(true)
    isDetected = canRange.isDetected
    supplyVoltageSignal = canRange.supplyVoltage

    canRange.configurator.apply(config)
  }

  private fun updateSignals() {
    BaseStatusSignal.refreshAll(ambientSignal, distance, isDetected, supplyVoltageSignal)
  }

  override fun updateInputs(inputs: CANRangeIO.CANRangeIOInputs) {
    updateSignals()
    inputs.supplyVoltage = supplyVoltageSignal.valueAsDouble.volts
    inputs.isDetected = isDetected.value
    inputs.ambientSignal = ambientSignal.valueAsDouble
    inputs.distance = distance.valueAsDouble.meters
  }
}
