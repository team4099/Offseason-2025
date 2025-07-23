import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
import org.team4099.lib.units.base.inAmperes
import org.team4099.lib.units.base.inCelsius
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
import org.team4099.lib.units.inDegreesPerSecond
import org.team4099.lib.units.inRotationsPerMinute
import org.team4099.lib.units.perMinute
import org.team4099.lib.units.perSecond

interface IntakeIO {
  class IntakeIOInputs : LoggableInputs {
    var pivotPosition = 0.0.degrees
    var pivotVelocity = 0.0.degrees.perSecond

    var pivotAppliedVoltage = 0.0.volts
    var pivotSupplyCurrent = 0.0.amps
    var pivotStatorCurrent = 0.0.amps
    var pivotTemp = 0.0.celsius

    var rollerVelocity = 0.0.rotations.perMinute

    var rollerAppliedVoltage = 0.0.volts
    var rollerSupplyCurrent = 0.0.amps
    var rollerStatorCurrent = 0.0.amps
    var rollerTemp = 0.0.celsius

    var isSimulating = false

    override fun toLog(table: LogTable?) {
      table?.put("armPositionDegrees", pivotPosition.inDegrees)
      table?.put("armVelocityDegreesPerSec", pivotVelocity.inDegreesPerSecond)

      table?.put("armAppliedVoltage", pivotAppliedVoltage.inVolts)

      table?.put("armSupplyCurrentAmps", pivotSupplyCurrent.inAmperes)

      table?.put("armStatorCurrentAmps", pivotStatorCurrent.inAmperes)

      table?.put("armTempCelsius", pivotTemp.inCelsius)

      table?.put("rollerVelocityRPM", rollerVelocity.inRotationsPerMinute)

      table?.put("rollerAppliedVoltage", rollerAppliedVoltage.inVolts)

      table?.put("rollerSupplyCurrentAmps", rollerSupplyCurrent.inAmperes)

      table?.put("rollerStatorCurrentAmps", rollerStatorCurrent.inAmperes)

      table?.put("rollerTempCelsius", rollerTemp.inCelsius)
    }

    override fun fromLog(table: LogTable?) {
      table?.get("armPositionDegrees", pivotPosition.inDegrees)?.let { pivotPosition = it.degrees }
      table?.get("armVelocityDegreesPerSec", pivotVelocity.inDegreesPerSecond)?.let {
        pivotVelocity = it.degrees.perSecond
      }
      table?.get("armAppliedVoltage", pivotAppliedVoltage.inVolts)?.let {
        pivotAppliedVoltage = it.volts
      }
      table?.get("armSupplyCurrentAmps", pivotSupplyCurrent.inAmperes)?.let {
        pivotSupplyCurrent = it.amps
      }
      table?.get("armStatorCurrentAmps", pivotStatorCurrent.inAmperes)?.let {
        pivotStatorCurrent = it.amps
      }
      table?.get("armTempCelsius", pivotTemp.inCelsius)?.let { pivotTemp = it.celsius }

      table?.get("rollerVelocityRPM", rollerVelocity.inRotationsPerMinute)?.let {
        rollerVelocity = it.rotations.perSecond
      }
      table?.get("rollerAppliedVoltage", rollerAppliedVoltage.inVolts)?.let {
        rollerAppliedVoltage = it.volts
      }
      table?.get("rollerSupplyCurrentAmps", rollerSupplyCurrent.inAmperes)?.let {
        rollerSupplyCurrent = it.amps
      }
      table?.get("rollerStatorCurrentAmps", rollerStatorCurrent.inAmperes)?.let {
        rollerStatorCurrent = it.amps
      }
      table?.get("rollerTempCelsius", rollerTemp.inCelsius)?.let { rollerTemp = it.celsius }
    }
  }

  fun updateInputs(inputs: IntakeIOInputs) {}

  /**
   * Sets the voltage of the roller motor but also checks to make sure the voltage doesn't exceed
   * limit
   *
   * @param voltage the voltage to set the motor to
   */
  fun setRollerVoltage(voltage: ElectricalPotential) {}

  /**
   * Sets the position of the arm motor, specifically the length of the arm
   *
   * @param position the position to set the arm to
   * @param feedforward changes voltages to compensate for external forces
   */
  fun setPivotPosition(position: Angle) {}

  /**
   * Sets the arm motor voltage, ensures the voltage is limited to battery voltage compensation
   *
   * @param voltage the voltage to set the arm motor to
   */
  fun setPivotVoltage(voltage: ElectricalPotential) {}

  /**
   * Updates the PID constants using the implementation controller
   *
   * @param kP accounts for linear error
   * @param kI accounts for integral error
   * @param kD accounts for derivative error
   */
  fun configPID(
    kP: ProportionalGain<Radian, Volt>,
    kI: IntegralGain<Radian, Volt>,
    kD: DerivativeGain<Radian, Volt>
  ) {}

  /** Sets the current encoder position to be the zero value */
  fun zeroEncoder() {}

  /**
   * Sets the roller motor brake mode
   *
   * @param brake if it brakes
   */
  fun setRollerBrakeMode(brake: Boolean) {}

  /**
   * Sets the roller motor brake mode
   *
   * @param brake if it brakes
   */
  fun setPivotBrakeMode(brake: Boolean) {}
}
