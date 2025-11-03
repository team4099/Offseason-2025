package com.team4099.robot2025.subsystems.climber

import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs
import org.team4099.lib.units.Value
import org.team4099.lib.units.Velocity
import org.team4099.lib.units.base.Current
import org.team4099.lib.units.base.Temperature
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
import org.team4099.lib.units.base.inAmperes
import org.team4099.lib.units.base.inCelsius
import org.team4099.lib.units.derived.AccelerationFeedforward
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.DerivativeGain
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.Force
import org.team4099.lib.units.derived.IntegralGain
import org.team4099.lib.units.derived.ProportionalGain
import org.team4099.lib.units.derived.Radian
import org.team4099.lib.units.derived.StaticFeedforward
import org.team4099.lib.units.derived.VelocityFeedforward
import org.team4099.lib.units.derived.Volt
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inDegrees
import org.team4099.lib.units.derived.inNewtons
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.newtons
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.inDegreesPerSecond
import org.team4099.lib.units.inDegreesPerSecondPerSecond
import org.team4099.lib.units.perSecond

interface ClimberIO {
  class ClimberInputs : LoggableInputs {
    var climberPosition: Angle = 0.0.degrees
    var climberVelocity: Value<Velocity<Radian>> = 0.0.degrees.perSecond
    var climberAcceleration: Value<Velocity<Velocity<Radian>>> = 0.0.degrees.perSecond.perSecond
    var climberTorque: Force = 0.0.newtons
    var climberAppliedVoltage: ElectricalPotential = 0.0.volts
    var climberDutyCycle: ElectricalPotential = 0.0.volts
    var climberStatorCurrent: Current = 0.0.amps
    var climberSupplyCurrent: Current = 0.0.amps
    var climberTemperature: Temperature = 0.0.celsius

    var rollersVelocity: Value<Velocity<Radian>> = 0.0.degrees.perSecond
    var rollersAppliedVoltage: ElectricalPotential = 0.0.volts
    var rollersStatorCurrent: Current = 0.0.amps
    var rollersSupplyCurrent: Current = 0.0.amps
    var rollersTemperature: Temperature = 0.0.celsius

    var isSimulated: Boolean = false

    override fun toLog(table: LogTable) {
      table.put("climberPositionDegrees", climberPosition.inDegrees)
      table.put("climberVelocityDegreesPerSecond", climberVelocity.inDegreesPerSecond)
      table.put(
        "climberAccelerationDegreesPerSecondPerSecond",
        climberAcceleration.inDegreesPerSecondPerSecond
      )
      table.put("climberTorqueNewtonMeters", climberTorque.inNewtons)
      table.put("climberAppliedVolts", climberAppliedVoltage.inVolts)
      table.put("climberDutyCycleVolts", climberDutyCycle.inVolts)
      table.put("climberStatorCurrentAmps", climberStatorCurrent.inAmperes)
      table.put("climberSupplyCurrentAmps", climberSupplyCurrent.inAmperes)
      table.put("climberTemperatureCelsius", climberTemperature.inCelsius)

      table.put("rollersVelocityDegreesPerSecond", rollersVelocity.inDegreesPerSecond)
      table.put("rollersAppliedVolts", rollersAppliedVoltage.inVolts)
      table.put("rollersStatorCurrentAmps", rollersStatorCurrent.inAmperes)
      table.put("rollersSupplyCurrentAmps", rollersSupplyCurrent.inAmperes)
      table.put("rollersTemperatureCelsius", rollersTemperature.inCelsius)
    }

    override fun fromLog(table: LogTable) {
      // ---------- CLIMBER ----------
      table.get("climberPositionDegrees", climberPosition.inDegrees).let {
        climberPosition = it.degrees
      }

      table.get("climberVelocityDegreesPerSecond", climberVelocity.inDegreesPerSecond).let {
        climberVelocity = it.degrees.perSecond
      }

      table.get(
        "climberAccelerationDegreesPerSecondPerSecond",
        climberAcceleration.inDegreesPerSecondPerSecond
      )
        .let { climberAcceleration = it.degrees.perSecond.perSecond }

      table.get("climberTorqueNewtonMeters", climberTorque.inNewtons).let {
        climberTorque = it.newtons
      }

      table.get("climberAppliedVolts", climberAppliedVoltage.inVolts).let {
        climberAppliedVoltage = it.volts
      }

      table.get("climberDutyCycleVolts", climberDutyCycle.inVolts).let {
        climberDutyCycle = it.volts
      }

      table.get("climberStatorCurrentAmps", climberStatorCurrent.inAmperes).let {
        climberStatorCurrent = it.amps
      }

      table.get("climberSupplyCurrentAmps", climberSupplyCurrent.inAmperes).let {
        climberSupplyCurrent = it.amps
      }

      table.get("climberTemperatureCelsius", climberTemperature.inCelsius).let {
        climberTemperature = it.celsius
      }

      // ---------- ROLLERS ----------
      table.get("rollersVelocityDegreesPerSecond", rollersVelocity.inDegreesPerSecond).let {
        rollersVelocity = it.degrees.perSecond
      }

      table.get("rollersAppliedVolts", rollersAppliedVoltage.inVolts).let {
        rollersAppliedVoltage = it.volts
      }

      table.get("rollersStatorCurrentAmps", rollersStatorCurrent.inAmperes).let {
        rollersStatorCurrent = it.amps
      }

      table.get("rollersSupplyCurrentAmps", rollersSupplyCurrent.inAmperes).let {
        rollersSupplyCurrent = it.amps
      }

      table.get("rollersTemperatureCelsius", rollersTemperature.inCelsius).let {
        rollersTemperature = it.celsius
      }
    }
  }

  fun updateInputs(inputs: ClimberInputs) {}

  fun setClimberVoltage(voltage: ElectricalPotential) {}

  fun setRollersVoltage(voltage: ElectricalPotential) {}

  fun setClimberPosition(position: Angle) {}

  fun setBrakeMode(climberBrake: Boolean, rollersBrake: Boolean) {}

  fun zeroEncoder() {}

  fun configClimberPID(
    kP: ProportionalGain<Radian, Volt>,
    kI: IntegralGain<Radian, Volt>,
    kD: DerivativeGain<Radian, Volt>
  ) {}

  fun configClimberFF(
    kG: ElectricalPotential,
    kS: StaticFeedforward<Volt>,
    kV: VelocityFeedforward<Radian, Volt>,
    kA: AccelerationFeedforward<Radian, Volt>
  ) {}
}
