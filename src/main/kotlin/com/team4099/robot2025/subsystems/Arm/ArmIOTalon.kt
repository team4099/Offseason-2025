package com.team4099.robot2025.subsystems.Arm

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.StatusSignal
import com.ctre.phoenix6.configs.CANcoderConfiguration
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.CANcoder
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue
import com.ctre.phoenix6.signals.GravityTypeValue
import com.ctre.phoenix6.signals.SensorDirectionValue
import com.team4099.lib.math.clamp
import com.team4099.robot2025.config.constants.ArmConstants
import com.team4099.robot2025.config.constants.Constants
import edu.wpi.first.units.measure.AngularAcceleration
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Voltage
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
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
import org.team4099.lib.units.derived.inRotations
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.inVoltsPerDegree
import org.team4099.lib.units.derived.inVoltsPerDegreePerSecond
import org.team4099.lib.units.derived.inVoltsPerDegreeSeconds
import org.team4099.lib.units.derived.inVoltsPerDegreesPerSecondPerSecond
import org.team4099.lib.units.derived.rotations
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.inDegreesPerSecond
import org.team4099.lib.units.inDegreesPerSecondPerSecond
import org.team4099.lib.units.perSecond
import edu.wpi.first.units.measure.Angle as WPIAngle
import edu.wpi.first.units.measure.Current as WPICurrent
import edu.wpi.first.units.measure.Temperature as WPITemp

object ArmIOTalon : ArmIO {
  private val armTalon: TalonFX = TalonFX(Constants.Arm.ARM_MOTOR_ID)

  private val absoluteEncoder: CANcoder = CANcoder(Constants.Arm.CANCODER_ID)

  private val absoluteEncoderConfig: CANcoderConfiguration = CANcoderConfiguration()

  private val motionMagicControl: MotionMagicVoltage = MotionMagicVoltage(-1337.degrees.inDegrees)

  private val armSensor =
    ctreAngularMechanismSensor(
      armTalon, ArmConstants.GEAR_RATIO, ArmConstants.VOLTAGE_COMPENSATION
    )

  private val configs: TalonFXConfiguration = TalonFXConfiguration()
  var slot0Configs = configs.Slot0

  var statorCurrentSignal: StatusSignal<WPICurrent>
  var supplyCurrentSignal: StatusSignal<WPICurrent>
  var tempSignal: StatusSignal<WPITemp>
  var dutyCycleSignal: StatusSignal<Double>
  var motorTorqueSignal: StatusSignal<WPICurrent>
  var motorVoltageSignal: StatusSignal<Voltage>
  var absoluteEncoderPositionSignal: StatusSignal<WPIAngle>
  var absoluteEncoderVelocitySignal: StatusSignal<AngularVelocity>
  var motorAcelSignal: StatusSignal<AngularAcceleration>

  init {
    armTalon.clearStickyFaults()
    absoluteEncoder.clearStickyFaults()

    configs.CurrentLimits.SupplyCurrentLimit = 40.0
    configs.CurrentLimits.SupplyCurrentLowerLimit = 20.0
    configs.CurrentLimits.StatorCurrentLimit = 40.0
    configs.CurrentLimits.SupplyCurrentLimitEnable = true
    configs.CurrentLimits.StatorCurrentLimitEnable = true

    configs.SoftwareLimitSwitch.ForwardSoftLimitEnable = false
    configs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true

    configs.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
      armSensor.positionToRawUnits(ArmConstants.MAX_ROTATION)

    configs.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
      armSensor.positionToRawUnits(ArmConstants.MIN_ROTATION)

    configs.MotionMagic.MotionMagicCruiseVelocity = ArmConstants.MAX_VELOCITY.inDegreesPerSecond
    configs.MotionMagic.MotionMagicAcceleration =
      ArmConstants.MAX_ACCELERATION.inDegreesPerSecondPerSecond

    configs.Slot0.GravityType = GravityTypeValue.Arm_Cosine

    configs.Feedback.FeedbackRemoteSensorID = absoluteEncoder.deviceID
    configs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder
    configs.Feedback.RotorToSensorRatio = 1.0 / ArmConstants.GEAR_RATIO
    configs.Feedback.SensorToMechanismRatio = 1.0 / ArmConstants.ENCODER_TO_MECHANISM_GEAR_RATIO

    statorCurrentSignal = armTalon.statorCurrent
    supplyCurrentSignal = armTalon.supplyCurrent
    tempSignal = armTalon.deviceTemp
    dutyCycleSignal = armTalon.dutyCycle
    motorTorqueSignal = armTalon.torqueCurrent
    motorVoltageSignal = armTalon.motorVoltage
    motorAcelSignal = armTalon.acceleration

    absoluteEncoderPositionSignal = absoluteEncoder.absolutePosition
    absoluteEncoderVelocitySignal = absoluteEncoder.velocity

    absoluteEncoderConfig.MagnetSensor.MagnetOffset = ArmConstants.ENCODER_ANGLE_OFFSET.inRotations
    absoluteEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive
    absoluteEncoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint =
      ArmConstants.CANCODER_DISCONTINUITY_POINT

    armTalon.configurator.apply(configs)
    absoluteEncoder.configurator.apply(absoluteEncoderConfig)

    zeroEncoder()
  }

  private fun updateSignals() {
    BaseStatusSignal.refreshAll(
      statorCurrentSignal,
      supplyCurrentSignal,
      tempSignal,
      dutyCycleSignal,
      motorTorqueSignal,
      motorVoltageSignal,
      absoluteEncoderPositionSignal,
      absoluteEncoderVelocitySignal,
      motorAcelSignal
    )
  }

  override fun updateInputs(inputs: ArmIO.ArmIOInputs) {
    updateSignals()

    inputs.armPosition =
      absoluteEncoderPositionSignal.valueAsDouble.rotations /
      ArmConstants.ENCODER_TO_MECHANISM_GEAR_RATIO
    inputs.armVelocity =
      absoluteEncoderVelocitySignal.valueAsDouble.rotations.perSecond /
      ArmConstants.ENCODER_TO_MECHANISM_GEAR_RATIO

    inputs.armTorque = armTalon.torqueCurrent.valueAsDouble
    inputs.armAppliedVoltage = motorVoltageSignal.valueAsDouble.volts
    inputs.armDutyCycle = dutyCycleSignal.valueAsDouble.volts
    inputs.armStatorCurrent = statorCurrentSignal.valueAsDouble.amps
    inputs.armSupplyCurrent = supplyCurrentSignal.valueAsDouble.amps
    inputs.armTemperature = tempSignal.valueAsDouble.celsius
    inputs.armAcceleration =
      (motorAcelSignal.valueAsDouble / ArmConstants.ENCODER_TO_MECHANISM_GEAR_RATIO)
        .rotations
        .perSecond
        .perSecond
  }

  override fun configPID(
    kP: ProportionalGain<Radian, Volt>,
    kI: IntegralGain<Radian, Volt>,
    kD: DerivativeGain<Radian, Volt>
  ) {
    slot0Configs.kP = kP.inVoltsPerDegree
    slot0Configs.kI = kI.inVoltsPerDegreeSeconds
    slot0Configs.kD = kD.inVoltsPerDegreePerSecond

    armTalon.configurator.apply(slot0Configs)
  }

  override fun configFF(
    kG: ElectricalPotential,
    kS: StaticFeedforward<Volt>,
    kV: VelocityFeedforward<Radian, Volt>,
    kA: AccelerationFeedforward<Radian, Volt>,
  ) {
    slot0Configs.kG = kG.inVolts
    slot0Configs.kS = kS.inVolts
    slot0Configs.kA = kA.inVoltsPerDegreesPerSecondPerSecond
    slot0Configs.kV = kV.inVoltsPerDegreePerSecond

    armTalon.configurator.apply(slot0Configs)
  }

  override fun zeroEncoder() {
    armTalon.setPosition(
      absoluteEncoderPositionSignal.valueAsDouble / ArmConstants.ENCODER_TO_MECHANISM_GEAR_RATIO
    )
  }

  override fun setVoltage(targetVoltage: ElectricalPotential) {
    val clampedVoltage =
      clamp(targetVoltage, -ArmConstants.VOLTAGE_COMPENSATION, ArmConstants.VOLTAGE_COMPENSATION)
    armTalon.setControl(VoltageOut(clampedVoltage.inVolts))
  }

  override fun setPosition(position: Angle) {
    val slotUsed = 0

    armTalon.setControl(
      motionMagicControl.withPosition(armSensor.positionToRawUnits(position)).withSlot(slotUsed)
    )
  }
}
