package com.team4099.robot2025.subsystems.climber

import com.team4099.lib.math.clamp
import com.team4099.robot2025.config.constants.ClimberConstants
import com.team4099.robot2025.config.constants.Constants
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.system.plant.LinearSystemId
import edu.wpi.first.wpilibj.simulation.FlywheelSim
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim
import org.team4099.lib.controller.PIDController
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
import org.team4099.lib.units.base.inMeters
import org.team4099.lib.units.base.inSeconds
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.DerivativeGain
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.IntegralGain
import org.team4099.lib.units.derived.ProportionalGain
import org.team4099.lib.units.derived.Radian
import org.team4099.lib.units.derived.Volt
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inKilogramsMeterSquared
import org.team4099.lib.units.derived.inRadians
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.newtons
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.derived.rotations
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.perMinute
import org.team4099.lib.units.perSecond

object ClimberIOSim : ClimberIO {
  private val climberSim: SingleJointedArmSim =
    SingleJointedArmSim(
      DCMotor.getKrakenX60(1),
      1 / ClimberConstants.GEAR_RATIO,
      ClimberConstants.INERTIA.inKilogramsMeterSquared,
      ClimberConstants.LENGTH.inMeters,
      ClimberConstants.FULLY_EXTENDED_ANGLE.inRadians,
      ClimberConstants.FULLY_CLIMBED_ANGLE.inRadians,
      true,
      0.0
    )

  private val rollersSim: FlywheelSim =
    FlywheelSim(
      LinearSystemId.createFlywheelSystem(
        DCMotor.getKrakenX60(1),
        ClimberConstants.Rollers.INERTIA.inKilogramsMeterSquared,
        1 / ClimberConstants.Rollers.GEAR_RATIO
      ),
      DCMotor.getKrakenX60(1),
      1 / ClimberConstants.Rollers.GEAR_RATIO
    )

  private var climberPIDController =
    PIDController(
      ClimberConstants.PID.KP_SIM, ClimberConstants.PID.KI_SIM, ClimberConstants.PID.KD_SIM
    )

  private var targetPosition: Angle = 0.0.degrees
  private var climberAppliedVoltage: ElectricalPotential = 0.0.volts
  private var rollersAppliedVoltage: ElectricalPotential = 0.0.volts

  override fun configClimberPID(
    kP: ProportionalGain<Radian, Volt>,
    kI: IntegralGain<Radian, Volt>,
    kD: DerivativeGain<Radian, Volt>
  ) {
    climberPIDController.setPID(kP, kI, kD)
  }

  override fun setClimberVoltage(voltage: ElectricalPotential) {
    val climberClampedVoltage =
      clamp(
        voltage,
        -ClimberConstants.CLIMBER_VOLTAGE_COMPENSATION,
        ClimberConstants.CLIMBER_VOLTAGE_COMPENSATION
      )

    climberSim.setInputVoltage(climberClampedVoltage.inVolts)
    climberAppliedVoltage = climberClampedVoltage
  }

  override fun setRollersVoltage(voltage: ElectricalPotential) {
    val rollersClampedVoltage =
      clamp(
        voltage,
        -ClimberConstants.Rollers.VOLTAGE_COMPENSATION,
        ClimberConstants.Rollers.VOLTAGE_COMPENSATION
      )

    rollersSim.setInputVoltage(rollersClampedVoltage.inVolts)
    rollersAppliedVoltage = rollersClampedVoltage
  }

  override fun setClimberPosition(position: Angle) {
    targetPosition = position
    // Rollers should constantly clasp to the bars while climber is moving
    setClimberVoltage(climberPIDController.calculate(climberSim.angleRads.radians, position))
    setRollersVoltage(ClimberConstants.Rollers.CLASP_VOLTAGE)
  }

  override fun updateInputs(inputs: ClimberIO.ClimberInputs) {
    climberSim.update(Constants.Universal.LOOP_PERIOD_TIME.inSeconds)
    inputs.climberPosition = climberSim.angleRads.radians
    inputs.climberVelocity = climberSim.velocityRadPerSec.radians.perSecond
    inputs.climberAppliedVoltage = climberAppliedVoltage
    inputs.climberStatorCurrent = climberSim.currentDrawAmps.amps
    inputs.climberSupplyCurrent = 0.0.amps
    inputs.climberTorque = 0.0.newtons
    inputs.climberTemperature = 0.0.celsius

    rollersSim.update(Constants.Universal.LOOP_PERIOD_TIME.inSeconds)
    inputs.rollersVelocity = rollersSim.angularVelocityRPM.rotations.perMinute
    inputs.rollersAppliedVoltage = rollersAppliedVoltage
    inputs.rollersStatorCurrent = rollersSim.currentDrawAmps.amps
    inputs.rollersSupplyCurrent = 0.0.amps
    inputs.rollersTemperature = 0.0.celsius
  }
}
