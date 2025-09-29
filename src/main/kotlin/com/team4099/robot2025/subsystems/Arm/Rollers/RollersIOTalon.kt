package com.team4099.robot2025.subsystems.Arm.Rollers

import com.ctre.phoenix.motorcontrol.NeutralMode
import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.StatusSignal
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import com.team4099.robot2025.config.constants.Constants
import com.team4099.robot2025.config.constants.RollersConstants
import edu.wpi.first.units.measure.AngularAcceleration
import edu.wpi.first.units.measure.Voltage
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
import org.team4099.lib.units.base.inAmperes
import org.team4099.lib.units.ctreAngularMechanismSensor
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.volts
import edu.wpi.first.units.measure.Current as WPICurrent
import edu.wpi.first.units.measure.Temperature as WPITemp

object RollersIOTalon : RollersIO {

  private val rollerTalon: TalonFX = TalonFX(Constants.ArmRollers.ARM_ROLLERS_MOTOR_ID)

  private val rollerConfig: TalonFXConfiguration = TalonFXConfiguration()

  private val configs: TalonFXConfiguration = TalonFXConfiguration()

  var statorCurrent: StatusSignal<WPICurrent>

  var supplyCurrent: StatusSignal<WPICurrent>

  var tempSignal: StatusSignal<WPITemp>

  var dutyCycleSignal: StatusSignal<Double>

  var motorTorque: StatusSignal<WPICurrent>

  var motorVoltage: StatusSignal<Voltage>

  var motorAccel: StatusSignal<AngularAcceleration>

  val voltageControl = VoltageOut(0.0.volts.inVolts)

  private var rollerSensor =
    ctreAngularMechanismSensor(
      rollerTalon, RollersConstants.GEAR_RATIO, RollersConstants.VOLTAGE_COMPENSATION
    )

  init {
    rollerTalon.clearStickyFaults()

    configs.CurrentLimits.SupplyCurrentLimit = RollersConstants.SUPPLY_CURRENT_LIMIT.inAmperes
    configs.CurrentLimits.StatorCurrentLimit = RollersConstants.STATOR_CURRENT_LIMIT.inAmperes
    configs.CurrentLimits.SupplyCurrentLimitEnable = true
    configs.CurrentLimits.StatorCurrentLimitEnable = true
    configs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive

    statorCurrent = rollerTalon.statorCurrent
    supplyCurrent = rollerTalon.supplyCurrent
    tempSignal = rollerTalon.deviceTemp
    dutyCycleSignal = rollerTalon.dutyCycle
    motorTorque = rollerTalon.torqueCurrent
    motorVoltage = rollerTalon.motorVoltage
    motorAccel = rollerTalon.acceleration

    rollerTalon.configurator.apply(configs)
  }

  override fun updateInputs(inputs: RollersIO.RollerInputs) {
    refreshStatusSignals()
    inputs.rollerAppliedVoltage = motorVoltage.valueAsDouble.volts
    inputs.rollerVelocity = rollerSensor.velocity
    inputs.rollerTemp = tempSignal.valueAsDouble.celsius
    inputs.rollerStatorCurrent = statorCurrent.valueAsDouble.amps
    inputs.rollerSupplyCurrent = supplyCurrent.valueAsDouble.amps
  }
  fun refreshStatusSignals() {
    BaseStatusSignal.refreshAll(
      supplyCurrent,
      statorCurrent,
      dutyCycleSignal,
      motorVoltage,
      motorAccel,
      tempSignal,
      motorTorque
    )
  }

  override fun setBrakeMode(brake: Boolean) {
    if (brake == true) {
      rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake
    } else {
      rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast
    }
  }

  override fun setVoltage(voltage: ElectricalPotential) {
    rollerTalon.setControl(voltageControl.withOutput(voltage.inVolts))
  }
}
