package com.team4099.robot2025.subsystems.superstructure.indexer

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.StatusSignal
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import com.team4099.robot2025.config.constants.Constants
import com.team4099.robot2025.config.constants.IndexerConstants
import edu.wpi.first.units.measure.Voltage
import org.team4099.lib.math.clamp
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
import org.team4099.lib.units.base.inAmperes
import org.team4099.lib.units.ctreAngularMechanismSensor
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.volts
import edu.wpi.first.units.measure.AngularAcceleration as WPIAngularAcceleration
import edu.wpi.first.units.measure.Current as WPICurrent
import edu.wpi.first.units.measure.Temperature as WPITemp

object IndexerIOTalon : IndexerIO {
  private val indexerTalon: TalonFX = TalonFX(Constants.Indexer.INDEXER_MOTOR_ID)
  private val indexerConfig: TalonFXConfiguration = TalonFXConfiguration()
  private val configs: TalonFXConfiguration = TalonFXConfiguration()

  var statorCurrent: StatusSignal<WPICurrent>

  var supplyCurrent: StatusSignal<WPICurrent>

  var tempSignal: StatusSignal<WPITemp>

  var dutyCycleSignal: StatusSignal<Double>

  var motorTorque: StatusSignal<WPICurrent>

  var motorVoltage: StatusSignal<Voltage>

  var motorAccel: StatusSignal<WPIAngularAcceleration>

  val voltageControl: VoltageOut = VoltageOut(0.0.volts.inVolts).withEnableFOC(true)

  private var indexerSensor =
    ctreAngularMechanismSensor(
      indexerTalon, IndexerConstants.GEAR_RATIO, IndexerConstants.VOLTAGE_COMPENSATION
    )

  init {
    indexerTalon.clearStickyFaults()

    configs.CurrentLimits.SupplyCurrentLimit = IndexerConstants.SUPPLY_CURRENT_LOWER_LIMIT.inAmperes
    configs.CurrentLimits.StatorCurrentLimit = IndexerConstants.STATOR_CURRENT_LIMIT.inAmperes
    //    configs.CurrentLimits.SupplyCurrentLowerLimit =
    // IndexerConstants.SUPPLY_CURRENT_LOWER_LIMIT.inAmperes
    //    configs.CurrentLimits.SupplyCurrentLowerTime =
    // IndexerConstants.SUPPLY_CURRENT_LOWER_TIME.inSeconds
    configs.CurrentLimits.SupplyCurrentLimitEnable = true
    configs.CurrentLimits.StatorCurrentLimitEnable = true
    configs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive

    statorCurrent = indexerTalon.statorCurrent
    supplyCurrent = indexerTalon.supplyCurrent
    tempSignal = indexerTalon.deviceTemp
    dutyCycleSignal = indexerTalon.dutyCycle
    motorTorque = indexerTalon.torqueCurrent
    motorVoltage = indexerTalon.motorVoltage
    motorAccel = indexerTalon.acceleration

    indexerTalon.configurator.apply(configs)
  }

  override fun updateInputs(inputs: IndexerIO.IndexerInputs) {
    BaseStatusSignal.refreshAll(
      supplyCurrent,
      statorCurrent,
      dutyCycleSignal,
      motorVoltage,
      motorAccel,
      tempSignal,
      motorTorque
    )

    inputs.indexerAppliedVoltage = motorVoltage.valueAsDouble.volts
    inputs.indexerVelocity = indexerSensor.velocity
    inputs.indexerTemp = tempSignal.valueAsDouble.celsius
    inputs.indexerStatorCurrent = statorCurrent.valueAsDouble.amps
    inputs.indexerSupplyCurrent = supplyCurrent.valueAsDouble.amps
  }

  override fun setBrakeMode(brake: Boolean) {
    indexerConfig.MotorOutput.NeutralMode =
      if (brake) {
        NeutralModeValue.Brake
      } else {
        NeutralModeValue.Coast
      }
  }

  override fun setVoltage(voltage: ElectricalPotential) {
    val clampedVoltage =
      clamp(
        voltage, -IndexerConstants.VOLTAGE_COMPENSATION, IndexerConstants.VOLTAGE_COMPENSATION
      )
    indexerTalon.setControl(voltageControl.withOutput(clampedVoltage.inVolts))
  }
}
