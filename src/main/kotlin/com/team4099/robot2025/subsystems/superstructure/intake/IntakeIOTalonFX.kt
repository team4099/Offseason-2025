package com.team4099.robot2025.subsystems.superstructure.intake

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
import com.team4099.robot2025.config.constants.Constants
import com.team4099.robot2025.config.constants.IntakeConstants
import org.ironmaple.simulation.IntakeSimulation
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
import org.team4099.lib.units.base.inAmperes
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
import org.team4099.lib.units.derived.inVoltsPerDegree
import org.team4099.lib.units.derived.inVoltsPerDegreePerSecond
import org.team4099.lib.units.derived.inVoltsPerDegreeSeconds
import org.team4099.lib.units.derived.inVoltsPerDegreesPerSecondPerSecond
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.inDegreesPerSecond
import org.team4099.lib.units.inDegreesPerSecondPerSecond
import edu.wpi.first.units.measure.Angle as WPIAngle
import edu.wpi.first.units.measure.Current as WPILibCurrent
import edu.wpi.first.units.measure.Temperature as WPILibTemperature
import edu.wpi.first.units.measure.Voltage as WPILibVoltage

object IntakeIOTalonFX : IntakeIO {
  private val pivotTalon: TalonFX = TalonFX(Constants.Intake.INTAKE_MOTOR_ID)
  private val pivotConfiguration: TalonFXConfiguration = TalonFXConfiguration()

  private val rollersTalon: TalonFX = TalonFX(Constants.Intake.Rollers.ROLLERS_MOTOR_ID)
  private val rollersConfiguration: TalonFXConfiguration = TalonFXConfiguration()

  private val pivotSensor =
    ctreAngularMechanismSensor(
      pivotTalon, IntakeConstants.PIVOT_GEAR_RATIO, IntakeConstants.VOLTAGE_COMPENSATION
    )

  private val rollerSensor =
    ctreAngularMechanismSensor(
      rollersTalon,
      IntakeConstants.Rollers.GEAR_RATIO,
      IntakeConstants.Rollers.VOLTAGE_COMPENSATION
    )

  var pivotPosition: StatusSignal<WPIAngle>
  var pivotAppliedVoltageStatusSignal: StatusSignal<WPILibVoltage>
  var pivotStatorCurrentStatusSignal: StatusSignal<WPILibCurrent>
  var pivotSupplyCurrentStatusSignal: StatusSignal<WPILibCurrent>
  var pivotTempStatusSignal: StatusSignal<WPILibTemperature>

  var rollerAppliedVoltageStatusSignal: StatusSignal<WPILibVoltage>
  var rollerStatorCurrentStatusSignal: StatusSignal<WPILibCurrent>
  var rollerSupplyCurrentStatusSignal: StatusSignal<WPILibCurrent>
  var rollerTempStatusSignal: StatusSignal<WPILibTemperature>

  val pivotVoltageControl: VoltageOut = VoltageOut(0.volts.inVolts)
  val pivotPositionControl: MotionMagicVoltage = MotionMagicVoltage(0.degrees.inDegrees)
  val rollerVoltageControl: VoltageOut = VoltageOut(0.volts.inVolts).withEnableFOC(true)

  override val intakeSimulation: IntakeSimulation? = null

  init {
    // Configure PID Values
    pivotConfiguration.Slot0.kP = IntakeConstants.PID.REAL_PIVOT_KP.inVoltsPerDegree
    pivotConfiguration.Slot0.kI = IntakeConstants.PID.REAL_PIVOT_KI.inVoltsPerDegreeSeconds
    pivotConfiguration.Slot0.kD = IntakeConstants.PID.REAL_PIVOT_KD.inVoltsPerDegreePerSecond
    pivotConfiguration.Slot0.GravityType = GravityTypeValue.Arm_Cosine

    // Configure Feedforward Values
    pivotConfiguration.Slot0.kG = IntakeConstants.PIVOT_KG.inVolts
    pivotConfiguration.Slot0.kS = IntakeConstants.PIVOT_KS.inVolts
    pivotConfiguration.Slot0.kA = IntakeConstants.PIVOT_KA.inVoltsPerDegreesPerSecondPerSecond
    pivotConfiguration.Slot0.kV = IntakeConstants.PIVOT_KV.inVoltsPerDegreePerSecond
    pivotConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive
    pivotConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake

    // configure arm
    pivotConfiguration.CurrentLimits.StatorCurrentLimit =
      IntakeConstants.STATOR_CURRENT_LIMIT.inAmperes
    pivotConfiguration.CurrentLimits.SupplyCurrentLimit =
      IntakeConstants.SUPPLY_CURRENT_LIMIT.inAmperes
    pivotConfiguration.CurrentLimits.StatorCurrentLimitEnable = true
    pivotConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true
    pivotConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive
    pivotConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake

    // configure motion magic
    pivotConfiguration.MotionMagic.MotionMagicAcceleration =
      IntakeConstants.MAX_ACCELERATION.inDegreesPerSecondPerSecond
    pivotConfiguration.MotionMagic.MotionMagicCruiseVelocity =
      IntakeConstants.MAX_VELOCITY.inDegreesPerSecond

    // Configure Softlimits
    pivotConfiguration.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
      pivotSensor.positionToRawUnits(IntakeConstants.PIVOT_MAX_ANGLE)
    pivotConfiguration.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
      pivotSensor.positionToRawUnits(IntakeConstants.PIVOT_MIN_ANGLE)

    pivotConfiguration.SoftwareLimitSwitch.ForwardSoftLimitEnable = true
    pivotConfiguration.SoftwareLimitSwitch.ReverseSoftLimitEnable = true

    // configure rollers
    rollersConfiguration.CurrentLimits.StatorCurrentLimit =
      IntakeConstants.Rollers.STATOR_CURRENT_LIMIT.inAmperes
    rollersConfiguration.CurrentLimits.SupplyCurrentLimit =
      IntakeConstants.Rollers.SUPPLY_CURRENT_LIMIT.inAmperes
    rollersConfiguration.CurrentLimits.StatorCurrentLimitEnable = true
    rollersConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true
    rollersConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive
    rollersConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake

    pivotTalon.configurator.apply(pivotConfiguration)
    rollersTalon.configurator.apply(rollersConfiguration)

    // sensor data
    pivotPosition = pivotTalon.position
    pivotAppliedVoltageStatusSignal = pivotTalon.motorVoltage
    pivotStatorCurrentStatusSignal = pivotTalon.statorCurrent
    pivotSupplyCurrentStatusSignal = pivotTalon.supplyCurrent
    pivotTempStatusSignal = pivotTalon.deviceTemp

    rollerAppliedVoltageStatusSignal = rollersTalon.motorVoltage
    rollerStatorCurrentStatusSignal = rollersTalon.statorCurrent
    rollerSupplyCurrentStatusSignal = rollersTalon.supplyCurrent
    rollerTempStatusSignal = rollersTalon.deviceTemp
  }

  fun refreshStatusSignals() {
    BaseStatusSignal.refreshAll(
      pivotPosition,
      pivotAppliedVoltageStatusSignal,
      pivotStatorCurrentStatusSignal,
      pivotSupplyCurrentStatusSignal,
      pivotTempStatusSignal,
      rollerAppliedVoltageStatusSignal,
      rollerStatorCurrentStatusSignal,
      rollerSupplyCurrentStatusSignal,
      rollerTempStatusSignal
    )
  }

  override fun updateInputs(inputs: IntakeIO.IntakeIOInputs) {
    refreshStatusSignals()
    inputs.pivotPosition = pivotSensor.position
    inputs.pivotVelocity = pivotSensor.velocity
    inputs.pivotAppliedVoltage = pivotAppliedVoltageStatusSignal.valueAsDouble.volts
    inputs.pivotStatorCurrent = pivotStatorCurrentStatusSignal.valueAsDouble.amps
    inputs.pivotSupplyCurrent = pivotSupplyCurrentStatusSignal.valueAsDouble.amps
    inputs.pivotTemp = pivotTempStatusSignal.valueAsDouble.celsius

    inputs.rollerVelocity = rollerSensor.velocity
    inputs.rollerAppliedVoltage = rollerAppliedVoltageStatusSignal.valueAsDouble.volts
    inputs.rollerStatorCurrent = rollerStatorCurrentStatusSignal.valueAsDouble.amps
    inputs.rollerSupplyCurrent = rollerSupplyCurrentStatusSignal.valueAsDouble.amps
    inputs.rollerTemp = rollerTempStatusSignal.valueAsDouble.celsius
    // inputs.beamBroken = beamBreakStatusSignal.value
  }

  override fun setPivotVoltage(voltage: ElectricalPotential) {
    pivotTalon.setControl(
      pivotVoltageControl
        .withEnableFOC(true)
        .withOutput(
          clamp(
            voltage,
            -IntakeConstants.VOLTAGE_COMPENSATION,
            IntakeConstants.VOLTAGE_COMPENSATION
          )
            .inVolts
        )
    )
  }

  override fun setPivotPosition(position: Angle) {
    pivotTalon.setControl(
      pivotPositionControl.withPosition(pivotSensor.positionToRawUnits(position))
    )
  }

  override fun zeroEncoder() {
    pivotTalon.setPosition(pivotSensor.positionToRawUnits(IntakeConstants.ZERO_OFFSET))
  }

  override fun configPID(
    kP: ProportionalGain<Radian, Volt>,
    kI: IntegralGain<Radian, Volt>,
    kD: DerivativeGain<Radian, Volt>
  ) {
    val PIDConfig = pivotConfiguration.Slot0
    PIDConfig.kP = kP.inVoltsPerDegree
    PIDConfig.kI = kI.inVoltsPerDegreeSeconds
    PIDConfig.kD = kD.inVoltsPerDegreePerSecond

    pivotTalon.configurator.apply(PIDConfig)
  }

  override fun setRollerVoltage(voltage: ElectricalPotential) {
    rollersTalon.setControl(rollerVoltageControl.withOutput(voltage.inVolts))
  }

  override fun setPivotBrakeMode(brake: Boolean) {
    if (brake) {
      pivotConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake
    } else {
      pivotConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast
    }

    pivotTalon.configurator.apply(pivotConfiguration)
  }

  override fun setRollerBrakeMode(brake: Boolean) {
    if (brake) {
      rollersConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake
    } else {
      rollersConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast
    }

    pivotTalon.configurator.apply(rollersConfiguration)
  }
}
