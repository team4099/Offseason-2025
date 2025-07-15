package com.team4099.robot2025.subsystems.Arm

import com.google.flatbuffers.Table
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
import org.team4099.lib.units.base.inAmperes
import org.team4099.lib.units.base.inCelsius
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.DerivativeGain
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.IntegralGain
import org.team4099.lib.units.derived.ProportionalGain
import org.team4099.lib.units.derived.Radian
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inDegrees
import org.team4099.lib.units.derived.inNewtons
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.newtons
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.inDegreesPerSecond
import org.team4099.lib.units.inDegreesPerSecondPerSecond
import org.team4099.lib.units.perSecond
import org.team4099.lib.units.derived.Volt


interface ArmIO {
    class ArmIOInputs:LoggableInputs {
        var armPosition = 0.0.degrees
        var armAbsoluteEncoderPosition = 0.0.degrees
        var armVelocity = 0.0.degrees.perSecond
        var armAppliedVoltage = 0.0.volts
        var armDutyCycle = 0.0.volts
        var armTorque = 0.0
        var armStatorCurrent = 0.0.amps
        var armSupplyCurrent = 0.0.amps
        var armTemperature = 0.0.celsius
        var armAcceleration = 0.0.degrees.perSecond.perSecond

        override fun toLog(table: LogTable) {
            table.put("armPosition", armPosition.inDegrees)
            table.put("armVelocity", armVelocity.inDegreesPerSecond)
            table.put("armAbsoluteEncoderPosition", armAbsoluteEncoderPosition.inDegrees)
            table.put("armAppliedVoltage", armAppliedVoltage.inVolts)
            table.put("armDutyCycle", armDutyCycle.inVolts)
            table.put("armStatorCurrent", armStatorCurrent.inAmperes)
            table.put("armSupplyCurrent", armSupplyCurrent.inAmperes)
            table.put("armTemperature", armTemperature.inCelsius)
            table.put("armAcceleration", armAcceleration.inDegreesPerSecondPerSecond)
        }

        override fun fromLog(table: LogTable) {
            table.get("armPosition", armPosition.inDegrees).let {armPosition = it.degrees}
            table.get("armVelocity", armVelocity.inDegreesPerSecond).let {armVelocity = it.degrees.perSecond}
            table.get("armAbsoluteEncoderPosition", armAbsoluteEncoderPosition.inDegrees).let {armAbsoluteEncoderPosition = it.degrees}
            table.get("armAppliedVoltage", armAppliedVoltage.inVolts).let {armAppliedVoltage = it.volts}
            table.get("armDutyCycle", armDutyCycle.inVolts).let {armDutyCycle = it.volts}
            table.get("armStatorCurrent",armDutyCycle.inVolts).let {armDutyCycle = it.volts}
            table.get("armSupplyCurrent", armSupplyCurrent.inAmperes).let {armSupplyCurrent = it.amps}
            table.get("armTemperature", armTemperature.inCelsius).let {armTemperature = it.celsius}
            table.get("armAcceleration", armAcceleration.inDegreesPerSecondPerSecond).let {armAcceleration = it.degrees.perSecond.perSecond}

        }

    }

    fun setArmBrakeMode(brake: Boolean){}

    fun setWristPosition(position: Angle, feedforward: ElectricalPotential) {}

    fun updateInputs (inputs: ArmIOInputs) {}

    fun configPID(
        kP: ProportionalGain <Radian, Volt>,
        kI: IntegralGain <Radian, Volt>,
        kD: DerivativeGain <Radian, Volt>
    ) {}
    fun configPIDSlot1(
        kP: ProportionalGain <Radian, Volt>,
        kI: IntegralGain <Radian, Volt>,
        kD: DerivativeGain <Radian, Volt>
    ) {}
    fun configPIDSlot2(
        kP: ProportionalGain <Radian, Volt>,
        kI: IntegralGain <Radian, Volt>,
        kD: DerivativeGain <Radian, Volt>
    ) {}
}

