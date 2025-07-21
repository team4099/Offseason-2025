package com.team4099.robot2025.subsystems.climber

import com.team4099.lib.math.clamp
import com.team4099.robot2025.config.constants.ClimberConstants
import com.team4099.robot2025.config.constants.Constants
import edu.wpi.first.math.system.plant.DCMotor
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
import org.team4099.lib.units.derived.newtons
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.perSecond

class ClimberIOSim : ClimberIO {
    private val climberSim: SingleJointedArmSim =
        SingleJointedArmSim(
            DCMotor.getKrakenX60(1),
            1 / ClimberConstants.GEAR_RATIO,
            ClimberConstants.INERTIA.inKilogramsMeterSquared,
            ClimberConstants.LENGTH.inMeters,
            0.degrees.inRadians,
            180.degrees.inRadians,
            true,
            0.0
        )

    private var climberPIDController =
        PIDController(
            ClimberConstants.PID.KP_SIM, ClimberConstants.PID.KI_SIM, ClimberConstants.PID.KD_SIM
        )

    private var targetPosition: Angle = 0.0.degrees
    private var appliedVoltage: ElectricalPotential = 0.0.volts

    override fun configPID(
        kP: ProportionalGain<Radian, Volt>,
        kI: IntegralGain<Radian, Volt>,
        kD: DerivativeGain<Radian, Volt>
    ) {
        climberPIDController.setPID(kP, kI, kD)
    }

    override fun setVoltage(voltage: ElectricalPotential) {
        val clampedVoltage =
            clamp(
                voltage, -ClimberConstants.VOLTAGE_COMPENSATION, ClimberConstants.VOLTAGE_COMPENSATION
            )
    }

    override fun setPosition(position: Angle, feedforward: ElectricalPotential) {
        targetPosition = position
        setVoltage(climberPIDController.calculate(climberSim.angleRads.radians, position))
    }

    override fun updateInputs(inputs: ClimberIO.ClimberInputs) {
        climberSim.update(Constants.Universal.LOOP_PERIOD_TIME.inSeconds)
        inputs.climberPosition = climberSim.angleRads.radians
        inputs.climberVelocity = climberSim.velocityRadPerSec.radians.perSecond
        inputs.climberAppliedVoltage = appliedVoltage
        inputs.climberStatorCurrent = climberSim.currentDrawAmps.amps
        inputs.climberSupplyCurrent = 0.0.amps
        inputs.climberTorque = 0.0.newtons
        inputs.climberTemperature = 0.0.celsius
    }
}