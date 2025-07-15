package com.team4099.robot2025.subsystems.Arm

import com.ctre.phoenix6.StatusSignal
import com.ctre.phoenix6.configs.MagnetSensorConfigs
import com.ctre.phoenix6.configs.Slot0Configs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage
import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.hardware.CANcoder
import com.ctre.phoenix6.hardware.ParentDevice.MapGenerator
import com.ctre.phoenix6.hardware.TalonFX
import com.team4099.robot2025.config.constants.ArmConstants
import com.team4099.robot2025.config.constants.Constants
import edu.wpi.first.units.measure.Angle as WPIAngle
import edu.wpi.first.units.measure.AngularAcceleration
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.units.measure.Temperature as WPITemp
import edu.wpi.first.units.measure.Current as WPICurrent
import edu.wpi.first.wpilibj.motorcontrol.Talon
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
import org.team4099.lib.units.ctreAngularMechanismSensor
import org.team4099.lib.units.derived.DerivativeGain
import org.team4099.lib.units.derived.IntegralGain
import org.team4099.lib.units.derived.ProportionalGain
import org.team4099.lib.units.derived.Radian
import org.team4099.lib.units.derived.Volt
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inDegrees
import org.team4099.lib.units.derived.rotations
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.perSecond

object RollersIOTalon: RollersIO {

    private val rollerTalon: TalonFX = TalonFX(Constants.Arm.ARM_MOTOR_ID)

    private val armConfig: TalonFXConfiguration = TalonFXConfiguration()

    private val configs: TalonFXConfiguration = TalonFXConfiguration()

    var statorCurrent: StatusSignal <WPICurrent>

    var supplyCurrent: StatusSignal <WPICurrent>

    var tempSignal: StatusSignal <WPITemp>

    var dutyCycleSignal: StatusSignal <Double>

    var motorTorque: StatusSignal <WPICurrent>

    var motorVoltage: StatusSignal <Voltage>

    var motorAccel: StatusSignal <AngularAcceleration>

    init {
        RollersIOTalon.clearStickyFaults()

        configs.CurrentLimits.SupplyCurrentLimit = 0.0
        configs.CurrentLimits.SupplyCurrentLowerLimit = 0.0
        configs.CurrentLimits.StatorCurrentLimit = 0.0
        configs.CurrentLimits.SupplyCurrentLimitEnable = true
        configs.CurrentLimits.StatorCurrentLimitEnable = true

        statorCurrent = rollerTalon.statorCurrent
        supplyCurrent = rollerTalon.supplyCurrent
        tempSignal = rollerTalon.deviceTemp
        dutyCycleSignal = rollerTalon.dutyCycle
        motorTorque = rollerTalon.torqueCurrent
        motorVoltage = rollerTalon.motorVoltage
        motorAccel = rollerTalon.acceleration
    }

    override fun updateInputs(inputs: ArmIO.ArmIOInputs) {
        inputs.arm
    }
}



