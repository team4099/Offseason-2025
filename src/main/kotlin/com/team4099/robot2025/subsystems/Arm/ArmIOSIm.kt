package com.team4099.robot2025.subsystems.Arm

import com.team4099.lib.math.clamp
import com.team4099.robot2025.config.constants.ArmConstants
import com.team4099.robot2025.config.constants.Constants
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim
import org.team4099.lib.controller.ArmFeedforward
import org.team4099.lib.controller.PIDController
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
import org.team4099.lib.units.base.inMeters
import org.team4099.lib.units.base.inSeconds
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
import org.team4099.lib.units.derived.inKilogramsMeterSquared
import org.team4099.lib.units.derived.inRadians
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.perSecond

object ArmIOSIm : ArmIO {
  val armSim =
    SingleJointedArmSim(
      DCMotor.getKrakenX60Foc(1),
      1 / ArmConstants.ENCODER_TO_MECHANISM_GEAR_RATIO,
      ArmConstants.ARM_MOMENT_OF_INERTIA.inKilogramsMeterSquared,
      ArmConstants.ARM_LENGTH.inMeters,
      ArmConstants.MIN_ROTATION.inRadians,
      ArmConstants.MAX_ROTATION.inRadians,
      true,
      0.0
    )

  var armTargetPos = -1337.degrees

  private val armController =
    PIDController(
      ArmConstants.PID.SIM_KP,
      ArmConstants.PID.SIM_KI,
      ArmConstants.PID.SIM_KD,
    )

  private var armFeedforward =
    ArmFeedforward(
      ArmConstants.PID.KS, ArmConstants.PID.KG, ArmConstants.PID.KV, ArmConstants.PID.KA
    )

  private var appliedVoltage = 0.0.volts

  override fun updateInputs(inputs: ArmIO.ArmIOInputs) {
    armSim.update(Constants.Universal.LOOP_PERIOD_TIME.inSeconds)

    inputs.armPosition = armTargetPos
    inputs.armVelocity = armSim.velocityRadPerSec.radians.perSecond
    inputs.armSupplyCurrent = 0.amps
    inputs.armAppliedVoltage = appliedVoltage
    inputs.armStatorCurrent = armSim.currentDrawAmps.amps
    inputs.armTemperature = 0.0.celsius

    inputs.isSimulated = true
  }

  override fun setVoltage(targetVoltage: ElectricalPotential) {
    val clampedVoltage =
      clamp(targetVoltage, -ArmConstants.VOLTAGE_COMPENSATION, ArmConstants.VOLTAGE_COMPENSATION)
    armSim.setInputVoltage(clampedVoltage.inVolts)
    appliedVoltage = clampedVoltage
  }

  override fun setPosition(position: Angle) {
    armTargetPos = position
    val feedback = armController.calculate(armSim.angleRads.radians, position)
    val feedforward =
      armFeedforward.calculate(
        armSim.angleRads.radians, armSim.velocityRadPerSec.radians.perSecond
      )
    setVoltage(feedback + feedforward)
  }

  override fun configPID(
    kP: ProportionalGain<Radian, Volt>,
    kI: IntegralGain<Radian, Volt>,
    kD: DerivativeGain<Radian, Volt>
  ) {
    armController.setPID(kP, kI, kD)
  }

  override fun configFF(
    kG: ElectricalPotential,
    kS: StaticFeedforward<Volt>,
    kV: VelocityFeedforward<Radian, Volt>,
    kA: AccelerationFeedforward<Radian, Volt>
  ) {
    armFeedforward = ArmFeedforward(kS, kG, kV, kA)
  }

  override fun zeroEncoder() {}
}
