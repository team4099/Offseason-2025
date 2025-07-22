package com.team4099.robot2025.subsystems.climber

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.StatusSignal
import com.ctre.phoenix6.configs.MotionMagicConfigs
import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.Slot0Configs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.GravityTypeValue
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import com.team4099.robot2025.config.constants.ClimberConstants
import com.team4099.robot2025.config.constants.Constants
import edu.wpi.first.units.measure.AngularAcceleration
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Current
import edu.wpi.first.units.measure.Temperature
import edu.wpi.first.units.measure.Voltage
import org.team4099.lib.controller.SimpleMotorFeedforward
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
import org.team4099.lib.units.base.inAmperes
import org.team4099.lib.units.ctreAngularMechanismSensor
import org.team4099.lib.units.derived.AccelerationFeedforward
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.DerivativeGain
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.IntegralGain
import org.team4099.lib.units.derived.ProportionalGain
import org.team4099.lib.units.derived.Radian
import org.team4099.lib.units.derived.StaticFeedforward
import org.team4099.lib.units.derived.VelocityFeedforward
import org.team4099.lib.units.derived.Volt
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inDegrees
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.newtons
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.perSecond

class ClimberIOTalon : ClimberIO {
    private val climberTalon: TalonFX = TalonFX(Constants.Climber.CLIMBER_MOTOR_ID)
    private val climberConfiguration: TalonFXConfiguration = TalonFXConfiguration()
    private val climberSensor = ctreAngularMechanismSensor(
        climberTalon,
        ClimberConstants.GEAR_RATIO,
        ClimberConstants.CLIMBER_VOLTAGE_COMPENSATION
    )

    private val rollersTalon: TalonFX = TalonFX(Constants.Rollers.ROLLERS_MOTOR_ID)
    private val rollersConfiguration: TalonFXConfiguration = TalonFXConfiguration()
    private val rollersSensor = ctreAngularMechanismSensor(
        rollersTalon,
        ClimberConstants.Rollers.GEAR_RATIO,
        ClimberConstants.Rollers.VOLTAGE_COMPENSATION
    )
    
    private val motionMagicConfig: MotionMagicConfigs = climberConfiguration.MotionMagic
    // -1337 symbolizes an "uninitialized" position to later be overwritten
    private val motionMagicControl: MotionMagicVoltage = MotionMagicVoltage((-1337).degrees.inDegrees)
    private var pidSlot0Configs: Slot0Configs = climberConfiguration.Slot0

    private var climberStatorCurrentSignal: StatusSignal<Current>
    private var climberSupplyCurrentSignal: StatusSignal<Current>
    private var climberTempSignal: StatusSignal<Temperature>
    private var climberDutyCycle: StatusSignal<Double>
    private var climberMotorVoltage: StatusSignal<Voltage>
    private var climberMotorTorque: StatusSignal<Current>
    private var climberMotorPosition: StatusSignal<edu.wpi.first.units.measure.Angle>
    private var climberMotorVelocity: StatusSignal<AngularVelocity>
    private var climberMotorAcceleration: StatusSignal<AngularAcceleration>

    private var rollersMotorVoltage: StatusSignal<Voltage>
    private var rollersStatorCurrentSignal: StatusSignal<Current>
    private var rollersSupplyCurrentSignal: StatusSignal<Current>
    private var rollersTempSignal: StatusSignal<Temperature>


    init {
        // ---------- CLIMBER ----------
        climberTalon.configurator.apply(TalonFXConfiguration())
        climberTalon.clearStickyFaults()

        climberConfiguration.Slot0.kP =
            climberSensor.proportionalPositionGainToRawUnits(ClimberConstants.PID.KP_REAL)
        climberConfiguration.Slot0.kI =
            climberSensor.integralPositionGainToRawUnits(ClimberConstants.PID.KI_REAL)
        climberConfiguration.Slot0.kD =
            climberSensor.derivativePositionGainToRawUnits(ClimberConstants.PID.KD_REAL)
        climberConfiguration.Slot0.GravityType = GravityTypeValue.Arm_Cosine

        climberConfiguration.CurrentLimits.StatorCurrentLimit = ClimberConstants.STATOR_CURRENT_LIMIT.inAmperes
        climberConfiguration.CurrentLimits.StatorCurrentLimitEnable = false
        climberConfiguration.CurrentLimits.SupplyCurrentLimit = ClimberConstants.SUPPLY_CURRENT_LIMIT.inAmperes
        climberConfiguration.CurrentLimits.SupplyCurrentLowerLimit = ClimberConstants.THRESHOLD_CURRENT_LIMIT.inAmperes
        climberConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true

        climberConfiguration.SoftwareLimitSwitch.ForwardSoftLimitEnable = true
        climberConfiguration.SoftwareLimitSwitch.ReverseSoftLimitEnable = true
        climberConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake
        climberConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive
        climberTalon.configurator.apply(climberConfiguration)

        climberStatorCurrentSignal = climberTalon.statorCurrent
        climberSupplyCurrentSignal = climberTalon.supplyCurrent
        climberDutyCycle = climberTalon.dutyCycle
        climberTempSignal = climberTalon.deviceTemp
        climberMotorVoltage = climberTalon.motorVoltage
        climberMotorTorque = climberTalon.torqueCurrent
        climberMotorPosition = climberTalon.position
        climberMotorVelocity = climberTalon.velocity
        climberMotorAcceleration = climberTalon.acceleration

        // ---------- ROLLERS ----------
        rollersTalon.configurator.apply(TalonFXConfiguration())
        rollersTalon.clearStickyFaults()

        rollersConfiguration.CurrentLimits.StatorCurrentLimit = ClimberConstants.Rollers.STATOR_CURRENT_LIMIT.inAmperes
        rollersConfiguration.CurrentLimits.SupplyCurrentLimit = ClimberConstants.Rollers.SUPPLY_CURRENT_LIMIT.inAmperes
        rollersConfiguration.CurrentLimits.StatorCurrentLimitEnable = false
        rollersConfiguration.CurrentLimits.SupplyCurrentLimitEnable = false
        rollersConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive
        rollersConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast
        rollersTalon.configurator.apply(rollersConfiguration)

        rollersMotorVoltage = rollersTalon.motorVoltage
        rollersStatorCurrentSignal = rollersTalon.statorCurrent
        rollersSupplyCurrentSignal = rollersTalon.supplyCurrent
        rollersTempSignal = rollersTalon.deviceTemp
    }

    override fun configClimberPID(
        kP: ProportionalGain<Radian, Volt>,
        kI: IntegralGain<Radian, Volt>,
        kD: DerivativeGain<Radian, Volt>
    ) {
        val climberPIDConfig = Slot0Configs()
        climberPIDConfig.kP = climberSensor.proportionalPositionGainToRawUnits(kP)
        climberPIDConfig.kI = climberSensor.integralPositionGainToRawUnits(kI)
        climberPIDConfig.kD = climberSensor.derivativePositionGainToRawUnits(kD)
        climberTalon.configurator.apply(climberPIDConfig)
    }

    override fun configClimberFF(
        kG: ElectricalPotential,
        kS: StaticFeedforward<Volt>,
        kV: VelocityFeedforward<Radian, Volt>,
        kA: AccelerationFeedforward<Radian, Volt>
    ) {
        pidSlot0Configs.kG = kG.inVolts
        pidSlot0Configs.kS = kS.inVolts
        pidSlot0Configs.kV = climberSensor.velocityFeedforwardToRawUnits(kV)
        pidSlot0Configs.kA = climberSensor.accelerationFeedforwardToRawUnits(kA)
        pidSlot0Configs.GravityType = GravityTypeValue.Arm_Cosine
        climberTalon.configurator.apply(pidSlot0Configs)
    }

    override fun zeroEncoder() {
        climberTalon.setPosition(0.0)
    }

    override fun setBrakeMode(climberBrake: Boolean, rollersBrake: Boolean) {
        val climberMotorOutputConfig = MotorOutputConfigs()
        val rollersMotorOutputConfig = MotorOutputConfigs()

        if (climberBrake) {
            climberMotorOutputConfig.NeutralMode = NeutralModeValue.Brake
        } else {
            climberMotorOutputConfig.NeutralMode = NeutralModeValue.Coast
        }

        if (rollersBrake) {
            rollersMotorOutputConfig.NeutralMode = NeutralModeValue.Brake
        } else {
            rollersMotorOutputConfig.NeutralMode = NeutralModeValue.Coast
        }

        climberTalon.configurator.apply(climberMotorOutputConfig)
        rollersTalon.configurator.apply(rollersMotorOutputConfig)
    }

    override fun setClimberVoltage(voltage: ElectricalPotential) {
        climberTalon.setControl(VoltageOut(voltage.inVolts))
    }

    override fun setRollersVoltage(voltage: ElectricalPotential) {
        rollersTalon.setControl(VoltageOut(voltage.inVolts))
    }

    override fun setClimberPosition(position: Angle) {
        climberTalon.setControl(
            motionMagicControl
                .withPosition(climberSensor.positionToRawUnits(position))
                .withSlot(0)
                .withLimitForwardMotion(true)
                .withLimitReverseMotion(true)
        )

        // Rollers should constantly clasp to the bars while climber is moving
        setRollersVoltage(ClimberConstants.Rollers.CLASP_VOLTAGE)
    }

    private fun updateSignals() {
        BaseStatusSignal.refreshAll(
            climberMotorPosition,
            climberMotorVelocity,
            climberMotorAcceleration,
            climberMotorTorque,
            climberMotorVoltage,
            climberDutyCycle,
            climberStatorCurrentSignal,
            climberSupplyCurrentSignal,
            climberTempSignal,
            rollersMotorVoltage,
            rollersStatorCurrentSignal,
            rollersSupplyCurrentSignal,
            rollersTempSignal
        )
    }

    override fun updateInputs(inputs: ClimberIO.ClimberInputs) {
        updateSignals()

        climberTalon.rotorPosition.refresh()
        climberTalon.position.refresh()

        inputs.climberPosition = climberSensor.position
        inputs.climberVelocity = climberSensor.velocity
        inputs.climberAcceleration = climberMotorAcceleration.valueAsDouble.degrees.perSecond.perSecond
        inputs.climberTorque = climberMotorTorque.valueAsDouble.newtons
        inputs.climberAppliedVoltage = climberMotorVoltage.valueAsDouble.volts
        inputs.climberDutyCycle = climberDutyCycle.valueAsDouble.volts
        inputs.climberStatorCurrent = climberStatorCurrentSignal.valueAsDouble.amps
        inputs.climberSupplyCurrent = climberSupplyCurrentSignal.valueAsDouble.amps
        inputs.climberTemperature = climberTempSignal.valueAsDouble.celsius

        inputs.rollersVelocity = rollersSensor.velocity
        inputs.rollersAppliedVoltage = rollersMotorVoltage.valueAsDouble.volts
        inputs.rollersStatorCurrent = rollersStatorCurrentSignal.valueAsDouble.amps
        inputs.rollersSupplyCurrent = rollersSupplyCurrentSignal.valueAsDouble.amps
        inputs.rollersTemperature = rollersTempSignal.valueAsDouble.celsius
    }
}