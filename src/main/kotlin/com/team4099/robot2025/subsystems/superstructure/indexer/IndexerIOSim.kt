package com.team4099.robot2025.subsystems.superstructure.indexer

import com.team4099.lib.math.clamp
import com.team4099.robot2025.config.constants.Constants
import com.team4099.robot2025.config.constants.IndexerConstants
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.system.plant.LinearSystemId
import edu.wpi.first.wpilibj.simulation.FlywheelSim
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
import org.team4099.lib.units.base.inSeconds
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.inKilogramsMeterSquared
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.rotations
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.perMinute

object IndexerIOSim : IndexerIO {
  private val indexerSim: FlywheelSim =
    FlywheelSim(
      LinearSystemId.createFlywheelSystem(
        DCMotor.getKrakenX60Foc(1),
        IndexerConstants.MOMENT_OF_INERTIA.inKilogramsMeterSquared,
        1 / IndexerConstants.GEAR_RATIO
      ),
      DCMotor.getKrakenX60Foc(1),
      1 / IndexerConstants.GEAR_RATIO
    )

  private var appliedVoltage = 0.0.volts

  override fun updateInputs(inputs: IndexerIO.IndexerInputs) {
    indexerSim.update(Constants.Universal.LOOP_PERIOD_TIME.inSeconds)

    inputs.indexerVelocity = indexerSim.angularVelocityRPM.rotations.perMinute
    inputs.indexerAppliedVoltage = appliedVoltage
    inputs.indexerStatorCurrent = indexerSim.currentDrawAmps.amps
    inputs.indexerSupplyCurrent = 0.0.amps
    inputs.indexerTemp = (-1337.0).celsius
    inputs.isSimulating = true
  }

  override fun setVoltage(voltage: ElectricalPotential) {
    val clampedVoltage =
      clamp(
        voltage, -IndexerConstants.VOLTAGE_COMPENSATION, IndexerConstants.VOLTAGE_COMPENSATION
      )
    indexerSim.setInputVoltage(clampedVoltage.inVolts)
    appliedVoltage = clampedVoltage
  }

  override fun setBrakeMode(brake: Boolean) {}
}
