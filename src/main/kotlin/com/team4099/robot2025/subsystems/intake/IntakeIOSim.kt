package com.team4099.robot2025.subsystems.intake

import com.team4099.lib.math.clamp
import com.team4099.robot2025.config.constants.Constants
import com.team4099.robot2025.config.constants.DrivetrainConstants
import com.team4099.robot2025.config.constants.IntakeConstants
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.system.plant.LinearSystemId
import edu.wpi.first.units.Units.Meters
import edu.wpi.first.wpilibj.simulation.FlywheelSim
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim
import org.ironmaple.simulation.IntakeSimulation
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation
import org.team4099.lib.controller.ProfiledPIDController
import org.team4099.lib.controller.TrapezoidProfile
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
import org.team4099.lib.units.derived.inKilogramsMeterSquared
import org.team4099.lib.units.derived.inRadians
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.derived.rotations
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.perMinute
import org.team4099.lib.units.perSecond

class IntakeIOSim(drivetrainSimulation: AbstractDriveTrainSimulation) : IntakeIO {

  private val armSim: SingleJointedArmSim =
    SingleJointedArmSim(
      DCMotor.getKrakenX60(1),
      1.0 / IntakeConstants.PIVOT_GEAR_RATIO,
      IntakeConstants.PIVOT_INERTIA.inKilogramsMeterSquared,
      IntakeConstants.PIVOT_LENGTH.inMeters,
      IntakeConstants.PIVOT_MIN_ANGLE.inRadians,
      IntakeConstants.PIVOT_MAX_ANGLE.inRadians,
      true,
      0.0
    )

  private val rollerSim: FlywheelSim =
    FlywheelSim(
      LinearSystemId.createFlywheelSystem(
        DCMotor.getKrakenX60(1),
        IntakeConstants.Rollers.INERTIA.inKilogramsMeterSquared,
        1 / IntakeConstants.Rollers.GEAR_RATIO
      ),
      DCMotor.getKrakenX60(1),
      1 / IntakeConstants.Rollers.GEAR_RATIO
    )

  private val armPIDController =
    ProfiledPIDController(
      IntakeConstants.PID.SIM_PIVOT_KP,
      IntakeConstants.PID.SIM_PIVOT_KI,
      IntakeConstants.PID.SIM_PIVOT_KD,
      TrapezoidProfile.Constraints(
        IntakeConstants.SIM_VELOCITY, IntakeConstants.SIM_ACCELERATION
      )
    )

  private var pivotAppliedVoltage = 0.0.volts
  private var rollersAppliedVoltage = 0.0.volts

  override val intakeSimulation =
    IntakeSimulation.OverTheBumperIntake(
      "Coral",
      drivetrainSimulation,
      Meters.of(DrivetrainConstants.DRIVETRAIN_WIDTH.inMeters),
      Meters.of(IntakeConstants.LENGTH_EXTENDED.inMeters),
      IntakeSimulation.IntakeSide.FRONT,
      1
    )

  init {
    armSim.setState(0.0, 0.0)
  }

  override fun updateInputs(inputs: IntakeIO.IntakeIOInputs) {
    armSim.update(Constants.Universal.LOOP_PERIOD_TIME.inSeconds)
    rollerSim.update(Constants.Universal.LOOP_PERIOD_TIME.inSeconds)

    inputs.pivotPosition = armSim.angleRads.radians
    inputs.pivotVelocity = armSim.velocityRadPerSec.radians.perSecond

    inputs.pivotAppliedVoltage = pivotAppliedVoltage
    inputs.pivotStatorCurrent = armSim.currentDrawAmps.amps
    inputs.pivotSupplyCurrent = 0.amps
    inputs.pivotTemp = 0.0.celsius

    inputs.rollerVelocity = rollerSim.angularVelocityRPM.rotations.perMinute
    inputs.rollerAppliedVoltage = rollersAppliedVoltage
    inputs.rollerStatorCurrent = rollerSim.currentDrawAmps.amps
    inputs.rollerSupplyCurrent = 0.0.amps
    inputs.rollerTemp = 25.0.celsius

    inputs.isSimulating = true
  }

  override fun setPivotVoltage(voltage: ElectricalPotential) {
    val clampedVoltage =
      clamp(voltage, -IntakeConstants.VOLTAGE_COMPENSATION, IntakeConstants.VOLTAGE_COMPENSATION)
    armSim.setInputVoltage(clampedVoltage.inVolts)
    pivotAppliedVoltage = clampedVoltage
  }

  override fun setPivotPosition(position: Angle) {
    val feedback = armPIDController.calculate(armSim.angleRads.radians, position)
    setPivotVoltage(feedback)
  }

  override fun setRollerVoltage(voltage: ElectricalPotential) {
    val clampedVoltage =
      clamp(
        voltage,
        -IntakeConstants.Rollers.VOLTAGE_COMPENSATION,
        IntakeConstants.Rollers.VOLTAGE_COMPENSATION
      )
    rollerSim.inputVoltage = clampedVoltage.inVolts
    rollersAppliedVoltage = clampedVoltage
  }

  override fun zeroEncoder() {}

  override fun setPivotBrakeMode(brake: Boolean) {}

  override fun configPID(
    kP: ProportionalGain<Radian, Volt>,
    kI: IntegralGain<Radian, Volt>,
    kD: DerivativeGain<Radian, Volt>
  ) {
    armPIDController.setPID(kP, kI, kD)
  }
}
