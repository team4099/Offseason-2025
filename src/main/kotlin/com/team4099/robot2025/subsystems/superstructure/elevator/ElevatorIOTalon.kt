package com.team4099.robot2025.subsystems.superstructure.elevator

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.StatusSignal
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.Follower
import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.GravityTypeValue
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import com.team4099.lib.math.clamp
import com.team4099.robot2025.config.constants.Constants
import com.team4099.robot2025.config.constants.ElevatorConstants
import com.team4099.robot2025.config.constants.ElevatorConstants.MAX_ACCELERATION
import com.team4099.robot2025.config.constants.ElevatorConstants.MAX_VELOCITY
import edu.wpi.first.units.measure.AngularVelocity
import org.littletonrobotics.junction.Logger
import org.team4099.lib.units.base.Length
import org.team4099.lib.units.base.Meter
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
import org.team4099.lib.units.base.inAmperes
import org.team4099.lib.units.base.inInches
import org.team4099.lib.units.base.inches
import org.team4099.lib.units.ctreLinearMechanismSensor
import org.team4099.lib.units.derived.AccelerationFeedforward
import org.team4099.lib.units.derived.DerivativeGain
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.IntegralGain
import org.team4099.lib.units.derived.ProportionalGain
import org.team4099.lib.units.derived.StaticFeedforward
import org.team4099.lib.units.derived.VelocityFeedforward
import org.team4099.lib.units.derived.Volt
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.inVoltsPerInch
import org.team4099.lib.units.derived.inVoltsPerInchPerSecond
import org.team4099.lib.units.derived.inVoltsPerInchSeconds
import org.team4099.lib.units.derived.inVoltsPerMetersPerSecondPerSecond
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.inInchesPerSecond
import org.team4099.lib.units.inInchesPerSecondPerSecond
import edu.wpi.first.units.measure.Current as WPILibCurrent
import edu.wpi.first.units.measure.Temperature as WPILibTemperature
import edu.wpi.first.units.measure.Voltage as WPILibVoltage

object ElevatorIOTalon : ElevatorIO {
  private val leaderTalon: TalonFX = TalonFX(Constants.Elevator.LEADER_MOTOR_ID)
  private val followerTalon: TalonFX = TalonFX(Constants.Elevator.FOLLOWER_MOTOR_ID)

  private val leaderConfigs: TalonFXConfiguration = TalonFXConfiguration()
  private val followerConfigs: TalonFXConfiguration = TalonFXConfiguration()
  private var leaderSlot0Configs = leaderConfigs.Slot0
  private var leaderSlot1Configs = leaderConfigs.Slot1
  private var followerSlot0Configs = leaderConfigs.Slot0
  private var followerSlot1Configs = leaderConfigs.Slot1

  private val motionMagicControl: MotionMagicVoltage = MotionMagicVoltage(-1337.inches.inInches)

  private val leaderSensor =
    ctreLinearMechanismSensor(
      leaderTalon,
      ElevatorConstants.GEAR_RATIO,
      ElevatorConstants.SPOOL_DIAMETER,
      ElevatorConstants.VOLTAGE_COMPENSATION
    )

  private val followerSensor =
    ctreLinearMechanismSensor(
      followerTalon,
      ElevatorConstants.GEAR_RATIO,
      ElevatorConstants.SPOOL_DIAMETER,
      ElevatorConstants.VOLTAGE_COMPENSATION
    )

  private var leaderStatorCurrentSignal: StatusSignal<WPILibCurrent>
  private var leaderSupplyCurrentSignal: StatusSignal<WPILibCurrent>
  private var leaderTempSignal: StatusSignal<WPILibTemperature>
  private var leaderDutyCycle: StatusSignal<Double>
  private var leaderPositionSignal: StatusSignal<edu.wpi.first.units.measure.Angle>
  private var leaderVelocitySignal: StatusSignal<AngularVelocity>

  private var followerStatorCurrentSignal: StatusSignal<WPILibCurrent>
  private var followerSupplyCurrentSignal: StatusSignal<WPILibCurrent>
  private var followerTempSignal: StatusSignal<WPILibTemperature>
  private var followerDutyCycle: StatusSignal<Double>
  private var motorVoltage: StatusSignal<WPILibVoltage>
  private var motorTorque: StatusSignal<WPILibCurrent>

  private var motionMagicTargetVelocity: StatusSignal<Double>
  private var motionMagicTargetPosition: StatusSignal<Double>

  init {
    leaderTalon.clearStickyFaults()
    followerTalon.clearStickyFaults()

    leaderConfigs.CurrentLimits.SupplyCurrentLimit =
      ElevatorConstants.SUPPLY_CURRENT_LIMIT.inAmperes
    leaderConfigs.CurrentLimits.StatorCurrentLimit =
      ElevatorConstants.STATOR_CURRENT_LIMIT.inAmperes
    leaderConfigs.CurrentLimits.StatorCurrentLimitEnable = true
    leaderConfigs.CurrentLimits.SupplyCurrentLimitEnable = true

    leaderConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true
    leaderConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true

    leaderConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
      leaderSensor.positionToRawUnits(ElevatorConstants.UPWARDS_EXTENSION_LIMIT)

    leaderConfigs.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
      leaderSensor.positionToRawUnits(ElevatorConstants.DOWNWARDS_EXTENSION_LIMIT)

    leaderConfigs.MotionMagic.MotionMagicCruiseVelocity = MAX_VELOCITY.inInchesPerSecond
    leaderConfigs.MotionMagic.MotionMagicAcceleration = MAX_ACCELERATION.inInchesPerSecondPerSecond
    leaderConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive

    followerTalon.setControl(Follower(Constants.Elevator.LEADER_MOTOR_ID, true))

    followerConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast
    leaderConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast

    followerConfigs.CurrentLimits.SupplyCurrentLimit =
      ElevatorConstants.SUPPLY_CURRENT_LIMIT.inAmperes
    followerConfigs.CurrentLimits.StatorCurrentLimit =
      ElevatorConstants.STATOR_CURRENT_LIMIT.inAmperes
    followerConfigs.CurrentLimits.StatorCurrentLimitEnable = true
    followerConfigs.CurrentLimits.SupplyCurrentLimitEnable = true

    followerConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true
    followerConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true

    followerConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
      followerSensor.positionToRawUnits(ElevatorConstants.UPWARDS_EXTENSION_LIMIT)

    followerConfigs.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
      followerSensor.positionToRawUnits(ElevatorConstants.DOWNWARDS_EXTENSION_LIMIT)

    followerConfigs.MotionMagic.MotionMagicCruiseVelocity = MAX_VELOCITY.inInchesPerSecond
    followerConfigs.MotionMagic.MotionMagicAcceleration =
      MAX_ACCELERATION.inInchesPerSecondPerSecond
    followerConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive

    leaderPositionSignal = leaderTalon.position
    leaderVelocitySignal = leaderTalon.velocity
    leaderStatorCurrentSignal = leaderTalon.statorCurrent
    leaderSupplyCurrentSignal = leaderTalon.supplyCurrent
    leaderTempSignal = leaderTalon.deviceTemp
    leaderDutyCycle = leaderTalon.dutyCycle

    followerStatorCurrentSignal = followerTalon.statorCurrent
    followerSupplyCurrentSignal = followerTalon.supplyCurrent
    followerTempSignal = followerTalon.deviceTemp
    followerDutyCycle = followerTalon.dutyCycle

    motorVoltage = leaderTalon.motorVoltage
    motorTorque = leaderTalon.torqueCurrent

    motionMagicTargetPosition = leaderTalon.closedLoopReference
    motionMagicTargetVelocity = leaderTalon.closedLoopReferenceSlope

    //    motionMagicTargetPosition.setUpdateFrequency(250.0)
    //    motionMagicTargetVelocity.setUpdateFrequency(250.0)

    leaderTalon.configurator.apply(leaderConfigs)
    followerTalon.configurator.apply(followerConfigs)
  }

  private fun updateSignals() {
    BaseStatusSignal.refreshAll(
      motorTorque,
      motorVoltage,
      leaderPositionSignal,
      leaderVelocitySignal,
      leaderTempSignal,
      leaderDutyCycle,
      leaderStatorCurrentSignal,
      leaderSupplyCurrentSignal,
      followerTempSignal,
      followerDutyCycle,
      followerStatorCurrentSignal,
      followerSupplyCurrentSignal,
      motionMagicTargetPosition,
      motionMagicTargetVelocity
    )
  }

  override fun updateInputs(inputs: ElevatorIO.ElevatorInputs) {
    updateSignals()

    inputs.elevatorPosition = leaderSensor.position
    inputs.elevatorVelocity = leaderSensor.velocity

    inputs.leaderTemperature = leaderTempSignal.valueAsDouble.celsius
    inputs.leaderSupplyCurrent = leaderSupplyCurrentSignal.valueAsDouble.amps
    inputs.leaderStatorCurrent = leaderStatorCurrentSignal.valueAsDouble.amps
    inputs.leaderAppliedVoltage = (leaderDutyCycle.valueAsDouble * 12).volts

    inputs.followerTemperature = followerTempSignal.valueAsDouble.celsius
    inputs.followerStatorCurrent = followerStatorCurrentSignal.valueAsDouble.amps
    inputs.followerSupplyCurrent = followerSupplyCurrentSignal.valueAsDouble.amps
    inputs.followerAppliedVoltage = (followerDutyCycle.valueAsDouble * 12).volts

    Logger.recordOutput(
      "Elevator/motionMagicPosition",
      motionMagicTargetPosition.value *
        ElevatorConstants.GEAR_RATIO *
        (Math.PI * ElevatorConstants.SPOOL_DIAMETER.inInches)
    )
    Logger.recordOutput(
      "Elevator/motionMagicVelocity",
      motionMagicTargetVelocity.value *
        ElevatorConstants.GEAR_RATIO *
        (Math.PI * ElevatorConstants.SPOOL_DIAMETER.inInches)
    )
  }

  override fun configPID(
    kP: ProportionalGain<Meter, Volt>,
    kI: IntegralGain<Meter, Volt>,
    kD: DerivativeGain<Meter, Volt>
  ) {
    leaderSlot0Configs.kP = kP.inVoltsPerInch
    leaderSlot0Configs.kI = kI.inVoltsPerInchSeconds
    leaderSlot0Configs.kD = kD.inVoltsPerInchPerSecond

    leaderSlot1Configs.kP = kP.inVoltsPerInch
    leaderSlot1Configs.kI = kI.inVoltsPerInchSeconds
    leaderSlot1Configs.kD = kD.inVoltsPerInchPerSecond

    followerSlot0Configs.kP = kP.inVoltsPerInch
    followerSlot0Configs.kI = kI.inVoltsPerInchSeconds
    followerSlot0Configs.kD = kD.inVoltsPerInchPerSecond

    followerSlot1Configs.kP = kP.inVoltsPerInch
    followerSlot1Configs.kI = kI.inVoltsPerInchSeconds
    followerSlot1Configs.kD = kD.inVoltsPerInchPerSecond

    leaderTalon.configurator.apply(leaderSlot0Configs)
    followerTalon.configurator.apply(followerSlot0Configs)
    leaderTalon.configurator.apply(leaderSlot1Configs)
    followerTalon.configurator.apply(followerSlot1Configs)
  }

  override fun configFF(
    kGFirstStage: ElectricalPotential,
    kGSecondStage: ElectricalPotential,
    kS: StaticFeedforward<Volt>,
    kV: VelocityFeedforward<Meter, Volt>,
    kA: AccelerationFeedforward<Meter, Volt>
  ) {
    leaderSlot0Configs.kG = kGFirstStage.inVolts
    leaderSlot0Configs.kS = kS.inVolts
    leaderSlot0Configs.kV = kV.inVoltsPerInchPerSecond
    leaderSlot0Configs.kA = kA.inVoltsPerMetersPerSecondPerSecond
    leaderSlot0Configs.GravityType = GravityTypeValue.Elevator_Static

    leaderSlot1Configs.kG = kGSecondStage.inVolts
    leaderSlot1Configs.kS = kS.inVolts
    leaderSlot1Configs.kV = kV.inVoltsPerInchPerSecond
    leaderSlot1Configs.kA = kA.inVoltsPerMetersPerSecondPerSecond
    leaderSlot1Configs.GravityType = GravityTypeValue.Elevator_Static

    followerSlot0Configs.kG = kGFirstStage.inVolts
    followerSlot0Configs.kS = kS.inVolts
    followerSlot0Configs.kV = kV.inVoltsPerInchPerSecond
    followerSlot0Configs.kA = kA.inVoltsPerMetersPerSecondPerSecond
    followerSlot0Configs.GravityType = GravityTypeValue.Elevator_Static

    followerSlot1Configs.kG = kGSecondStage.inVolts
    followerSlot1Configs.kS = kS.inVolts
    followerSlot1Configs.kV = kV.inVoltsPerInchPerSecond
    followerSlot1Configs.kA = kA.inVoltsPerMetersPerSecondPerSecond
    followerSlot1Configs.GravityType = GravityTypeValue.Elevator_Static

    leaderTalon.configurator.apply(leaderSlot0Configs)
    followerTalon.configurator.apply(leaderSlot0Configs)
    leaderTalon.configurator.apply(followerSlot1Configs)
    followerTalon.configurator.apply(followerSlot1Configs)
  }

  override fun setPosition(position: Length) {
    val slotUsed =
      if (leaderSensor.position > ElevatorConstants.FIRST_STAGE_HEIGHT) {
        1
      } else {
        0
      }

    leaderTalon.setControl(
      motionMagicControl
        .withPosition(leaderSensor.positionToRawUnits(position))
        .withSlot(slotUsed)
    )
  }

  override fun setVoltage(targetVoltage: ElectricalPotential) {
    leaderTalon.setControl(
      VoltageOut(
        clamp(
          targetVoltage,
          -ElevatorConstants.VOLTAGE_COMPENSATION,
          ElevatorConstants.VOLTAGE_COMPENSATION
        )
          .inVolts
      )
    )
  }

  override fun zeroEncoder() {
    leaderTalon.setPosition(0.0)
    followerTalon.setPosition(0.0)
  }
}
