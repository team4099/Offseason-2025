package com.team4099.robot2025.subsystems.indexer

import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
import org.team4099.lib.units.base.inAmperes
import org.team4099.lib.units.base.inCelsius
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.inDegreesPerSecond
import org.team4099.lib.units.perSecond

interface IndexerIO {
  class IndexerInputs : LoggableInputs {
    var indexerAppliedVoltage = 0.0.volts
    var indexerVelocity = 0.0.degrees.perSecond
    var indexerSupplyCurrent = 0.0.amps
    var indexerStatorCurrent = 0.0.amps
    var indexerTemp = 0.0.celsius

    var isSimulating = false

    override fun toLog(table: LogTable) {
      table.put("indexerAppliedVoltage", indexerAppliedVoltage.inVolts)
      table.put("indexerVelocity", indexerVelocity.inDegreesPerSecond)
      table.put("indexerSupplyCurrent", indexerSupplyCurrent.inAmperes)
      table.put("indexerStatorCurrent", indexerStatorCurrent.inAmperes)
      table.put("indexerTemp", indexerTemp.inCelsius)
      table.put("isSimulating", isSimulating)
    }

    override fun fromLog(table: LogTable) {
      table.get("indexerAppliedVoltage", indexerAppliedVoltage.inVolts).let {
        indexerAppliedVoltage = it.volts
      }
      table.get("indexerVelocity", indexerVelocity.inDegreesPerSecond).let {
        indexerVelocity = it.degrees.perSecond
      }
      table.get("indexerSupplyCurrent", indexerSupplyCurrent.inAmperes).let {
        indexerSupplyCurrent = it.amps
      }
      table.get("indexerStatorCurrent", indexerStatorCurrent.inAmperes).let {
        indexerStatorCurrent = it.amps
      }
      table.get("indexerAppliedVoltage", indexerAppliedVoltage.inVolts).let {
        indexerAppliedVoltage = it.volts
      }
      table.get("isSimulating", isSimulating)
    }
  }
  fun updateInputs(inputs: IndexerInputs) {}
  fun setVoltage(voltage: ElectricalPotential) {}
  fun setBrakeMode(brake: Boolean) {}
}
