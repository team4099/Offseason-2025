package com.team4099.robot2025.subsystems.Arm

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.StatusSignal
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.GravityTypeValue
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
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
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.inVoltsPerDegree
import org.team4099.lib.units.derived.inVoltsPerDegreePerSecond
import org.team4099.lib.units.derived.inVoltsPerDegreeSeconds
import org.team4099.lib.units.derived.inVoltsPerDegreesPerSecondPerSecond
import org.team4099.lib.units.derived.rotations
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.perSecond
import edu.wpi.first.units.measure.Angle as WPIAngle
import edu.wpi.first.units.measure.Current as WPICurrent
import edu.wpi.first.units.measure.Temperature as WPITemp

object ArmIOTalon : ArmIO {
  private val armTalon: TalonFX = TalonFX(Constants.Arm.ARM_MOTOR_ID)
  //  private val absoluteEncoder: CANcoder = CANcoder(Constants.Arm.CANCODER_ID)

  //  private val absoluteEncoderConfig: CANcoderConfiguration = CANcoderConfiguration()

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
  //  var absoluteEncoderPositionSignal: StatusSignal<WPIAngle>
  //  var absoluteEncoderAbsolutePositionSignal: StatusSignal<WPIAngle>
  //  var absoluteEncoderVelocitySignal: StatusSignal<AngularVelocity>
  var motorAccelSignal: StatusSignal<AngularAcceleration>
  var rotorPositionSignal: StatusSignal<WPIAngle>
  var rotorVelocitySignal: StatusSignal<AngularVelocity>

  //  var faultFusedSensorOutOfSync: StatusSignal<Boolean> =
  // armTalon.getFault_FusedSensorOutOfSync(false)
  //  var stickyFaultFusedSensorOutOfSync: StatusSignal<Boolean> =
  // armTalon.getStickyFault_FusedSensorOutOfSync(false)
  //  var faultRemoteSensorInvalid: StatusSignal<Boolean> =
  // armTalon.getFault_RemoteSensorDataInvalid(false)
  //  var stickyFaultRemoteSensorInvalid: StatusSignal<Boolean> =
  // armTalon.getStickyFault_RemoteSensorDataInvalid(false)

  init {
    armTalon.clearStickyFaults()
    //    absoluteEncoder.clearStickyFaults()

    //    absoluteEncoderConfig.MagnetSensor.MagnetOffset = 0.187 //
    // ArmConstants.ENCODER_ANGLE_OFFSET.inRotations
    //    absoluteEncoderConfig.MagnetSensor.SensorDirection =
    // SensorDirectionValue.Clockwise_Positive
    //    absoluteEncoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint =
    //      ArmConstants.CANCODER_DISCONTINUITY_POINT.inRotations

    //    absoluteEncoder.clearStickyFaults()

    //    absoluteEncoder.configurator.apply(absoluteEncoderConfig)

    //    absoluteEncoder.clearStickyFaults()

    //    do {
    //      absoluteEncoder.modulePosition.refresh()
    //      Logger.recordOutput("Arm/cancoderPublishing", absoluteEncoder.modulePosition.status.isOK)
    //    } while (!absoluteEncoder.modulePosition.status.isOK)

    //    absoluteEncoderPositionSignal = absoluteEncoder.modulePosition
    //    absoluteEncoderAbsolutePositionSignal = absoluteEncoder.absolutePosition
    //    absoluteEncoderVelocitySignal = absoluteEncoder.velocity

    configs.CurrentLimits.SupplyCurrentLimit = 40.0
    configs.CurrentLimits.StatorCurrentLimit = 40.0
    configs.CurrentLimits.SupplyCurrentLimitEnable = true
    configs.CurrentLimits.StatorCurrentLimitEnable = true
    configs.MotorOutput.NeutralMode = NeutralModeValue.Brake
    configs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive

    configs.MotionMagic.MotionMagicCruiseVelocity = ArmConstants.MAX_VELOCITY
    configs.MotionMagic.MotionMagicAcceleration = ArmConstants.MAX_ACCELERATION

    configs.Slot0.GravityType = GravityTypeValue.Arm_Cosine

    //    configs.Feedback.FeedbackRemoteSensorID = absoluteEncoder.deviceID
    //    configs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder
    //    configs.Feedback.RotorToSensorRatio = 1.0 / ArmConstants.GEAR_RATIO
    //    configs.Feedback.SensorToMechanismRatio = 1.0 /
    // ArmConstants.ENCODER_TO_MECHANISM_GEAR_RATIO

    armTalon.clearStickyFaults()

    armTalon.configurator.apply(configs)

    armTalon.clearStickyFaults()

    rotorPositionSignal = armTalon.position
    rotorVelocitySignal = armTalon.velocity
    statorCurrentSignal = armTalon.statorCurrent
    supplyCurrentSignal = armTalon.supplyCurrent
    tempSignal = armTalon.deviceTemp
    dutyCycleSignal = armTalon.dutyCycle
    motorTorqueSignal = armTalon.torqueCurrent
    motorVoltageSignal = armTalon.motorVoltage
    motorAccelSignal = armTalon.acceleration

    armTalon.clearStickyFaults()
    //    absoluteEncoder.clearStickyFaults()

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
      //      absoluteEncoderPositionSignal,
      //      absoluteEncoderAbsolutePositionSignal,
      //      absoluteEncoderVelocitySignal,
      motorAccelSignal,
      rotorPositionSignal,
      rotorVelocitySignal,
      //      faultFusedSensorOutOfSync,
      //      stickyFaultFusedSensorOutOfSync,
      //      faultRemoteSensorInvalid,
      //      stickyFaultRemoteSensorInvalid
    )
  }

  override fun updateInputs(inputs: ArmIO.ArmIOInputs) {
    updateSignals()
    //    zeroEncoder()

    //    Logger.recordOutput("Arm/faultFusedSensorOutOfSync", faultFusedSensorOutOfSync.value)
    //    Logger.recordOutput("Arm/stickyFaultFusedSensorOutOfSync",
    // stickyFaultFusedSensorOutOfSync.value)
    //    Logger.recordOutput("Arm/faultRemoteSensorInvalid", faultRemoteSensorInvalid.value)
    //    Logger.recordOutput("Arm/stickyFaultRemoteSensorInvalid",
    // stickyFaultRemoteSensorInvalid.value)

    inputs.armPosition = armSensor.position
    inputs.armVelocity = armSensor.velocity
    //    inputs.armAbsoluteEncoderPosition = absoluteEncoderPositionSignal.valueAsDouble.rotations
    //    inputs.armAbsoluteEncoderAbsolutePosition =
    //      absoluteEncoderAbsolutePositionSignal.valueAsDouble.rotations
    inputs.armTorque = armTalon.torqueCurrent.valueAsDouble
    inputs.armAppliedVoltage = motorVoltageSignal.valueAsDouble.volts
    inputs.armDutyCycle = dutyCycleSignal.valueAsDouble
    inputs.armStatorCurrent = statorCurrentSignal.valueAsDouble.amps
    inputs.armSupplyCurrent = supplyCurrentSignal.valueAsDouble.amps
    inputs.armTemperature = tempSignal.valueAsDouble.celsius
    inputs.armAcceleration =
      (motorAccelSignal.valueAsDouble / ArmConstants.ENCODER_TO_MECHANISM_GEAR_RATIO)
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
    armTalon.setPosition(armSensor.positionToRawUnits(ArmConstants.ANGLES.MANUAL_ZERO_ANGLE))
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
