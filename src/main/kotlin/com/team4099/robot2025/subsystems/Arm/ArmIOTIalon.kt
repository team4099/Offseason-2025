package com.team4099.robot2025.subsystems.Arm

import com.ctre.phoenix6.StatusSignal
import com.ctre.phoenix6.configs.MagnetSensorConfigs
import com.ctre.phoenix6.configs.Slot0Configs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage
import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.controls.VoltageOut
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
import org.team4099.lib.units.Value
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.ctreAngularMechanismSensor
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.DerivativeGain
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.IntegralGain
import org.team4099.lib.units.derived.ProportionalGain
import org.team4099.lib.units.derived.Radian
import org.team4099.lib.units.derived.Volt
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inDegrees
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.rotations
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.perSecond


object ArmIOTIalon: ArmIO {
    private val armTalon: TalonFX = TalonFX(Constants.Arm.ARM_MOTOR_ID);

    private val armConfig: TalonFXConfiguration = TalonFXConfiguration();

    private val absoluteEncoder: CANcoder = CANcoder(Constants.Arm.CANCODER_ID);

    private val absoluteEncoderConfig: MagnetSensorConfigs = MagnetSensorConfigs();

    private val motionMagicControl: MotionMagicVoltage = MotionMagicVoltage(-1337.degrees.inDegrees);

    private val armSensor = ctreAngularMechanismSensor(armTalon, 1.0, ArmConstants.VOLTAGE_COMPENSATION);

    private val configs: TalonFXConfiguration = TalonFXConfiguration();
    private var slot0Configs = configs.Slot0
    private var slot2Configs = configs.Slot0
    private var slot3Configs = configs.Slot0

    var statorCurrentSignal: StatusSignal <WPICurrent>
    var supplyCurrentSignal: StatusSignal <WPICurrent>
    var tempSignal: StatusSignal <WPITemp>
    var dutyCycleSignal: StatusSignal <Double>
    var motorTorqueSignal: StatusSignal <WPICurrent>
    var motorVoltageSignal: StatusSignal<Voltage>
    var absoluteEncoderSignal: StatusSignal<WPIAngle>
    var motorAcelSignal: StatusSignal<AngularAcceleration>

    init {
        armTalon.clearStickyFaults()

        configs.CurrentLimits.SupplyCurrentLimit = 0.0
        configs.CurrentLimits.SupplyCurrentLowerLimit =0.0
        configs.CurrentLimits.StatorCurrentLimit =0.0
        configs.CurrentLimits.SupplyCurrentLimitEnable = true
        configs.CurrentLimits.StatorCurrentLimitEnable = true

        configs.MotionMagic.MotionMagicCruiseVelocity = 0.0
        configs.MotionMagic.MotionMagicAcceleration = 0.0

        statorCurrentSignal = armTalon.statorCurrent
        supplyCurrentSignal = armTalon.supplyCurrent
        tempSignal = armTalon.deviceTemp
        dutyCycleSignal = armTalon.dutyCycle
        motorTorqueSignal = armTalon.torqueCurrent
        motorVoltageSignal = armTalon.motorVoltage
        motorAcelSignal = armTalon.acceleration

        absoluteEncoderSignal = absoluteEncoder.position
    }

    override fun updateInputs(inputs: ArmIO.ArmIOInputs){
        inputs.armPosition = absoluteEncoderSignal.valueAsDouble.degrees
        inputs.armVelocity = absoluteEncoder.velocity.valueAsDouble.degrees.perSecond
        inputs.armTorque = armTalon.torqueCurrent.valueAsDouble
        inputs.armAppliedVoltage = motorVoltageSignal.valueAsDouble.volts
        inputs.armDutyCycle = dutyCycleSignal.valueAsDouble.volts
        inputs.armStatorCurrent = statorCurrentSignal.valueAsDouble.amps
        inputs.armSupplyCurrent  = supplyCurrentSignal.valueAsDouble.amps
        inputs.armTemperature = tempSignal.valueAsDouble.celsius
        inputs.armAcceleration = (motorAcelSignal.valueAsDouble * ArmConstants.ENCODER_TO_MECHANISM_GEAR_RATIO).rotations.perSecond.perSecond
    }

    override fun configPID(
        kP: ProportionalGain<Radian, Volt>,
        kI: IntegralGain<Radian, Volt>,
        kD: DerivativeGain<Radian, Volt>
    ) {
        val PIDConfig = Slot0Configs()
        PIDConfig.kP = armSensor.proportionalPositionGainToRawUnits(kP)
        PIDConfig.kI = armSensor.integralPositionGainToRawUnits(kI)
        PIDConfig.kD = armSensor.derivativePositionGainToRawUnits(kD)
    }

    override fun configPIDSlot1(
        kP: ProportionalGain<Radian, Volt>,
        kI: IntegralGain<Radian, Volt>,
        kD: DerivativeGain<Radian, Volt>
    ) {
        val PIDConfig = Slot0Configs()
        PIDConfig.kP = armSensor.proportionalPositionGainToRawUnits(kP)
        PIDConfig.kI = armSensor.integralPositionGainToRawUnits(kI)
        PIDConfig.kD = armSensor.derivativePositionGainToRawUnits(kD)
    }

    override fun zeroEncoder() {
        absoluteEncoder.setPosition(0.0)
    }

    override fun setVoltage(targetVoltage: ElectricalPotential) {
        armTalon.setControl(VoltageOut(targetVoltage.inVolts))
    }

    override fun setPosition(position: Angle) {
        val slotUsed = 0

        armTalon.setControl(
            motionMagicControl
                .withPosition(armSensor.positionToRawUnits(position))
                .withSlot(slotUsed)
        )
    }

}