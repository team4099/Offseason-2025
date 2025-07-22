package com.team4099.robot2025.subsystems.climber

import com.team4099.robot2025.config.constants.ClimberConstants
import com.team4099.robot2025.subsystems.superstructure.Request
import com.team4099.robot2025.util.CustomLogger
import edu.wpi.first.wpilibj.RobotBase
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.volts

class Climber(private val io: ClimberIO) {
  private val inputs: ClimberIO.ClimberInputs = ClimberIO.ClimberInputs()

  private var currentState: ClimberState = ClimberState.UNINITIALIZED
  private var currentRequest: Request.ClimberRequest = Request.ClimberRequest.Home()
    set(value) {
      when (value) {
        is Request.ClimberRequest.OpenLoop -> {
          climberTargetVoltage = value.climberVoltage
          rollersTargetVoltage = value.rollersVoltage
        }
        else -> {}
      }
      field = value
    }

  private var climberTargetVoltage: ElectricalPotential = 0.0.volts
  private var rollersTargetVoltage: ElectricalPotential = 0.0.volts
  var isAtTargetedPosition: Boolean =
    (inputs.climberPosition - 90.degrees).absoluteValue <=
      ClimberConstants.CLIMBER_TARGET_TOLERANCE

  init {
    if (RobotBase.isReal()) {
      ClimberTunableValues.kP.initDefault(ClimberConstants.PID.KP_REAL)
      ClimberTunableValues.kI.initDefault(ClimberConstants.PID.KI_REAL)
      ClimberTunableValues.kD.initDefault(ClimberConstants.PID.KD_REAL)
    } else {
      ClimberTunableValues.kP.initDefault(ClimberConstants.PID.KP_SIM)
      ClimberTunableValues.kI.initDefault(ClimberConstants.PID.KI_SIM)
      ClimberTunableValues.kD.initDefault(ClimberConstants.PID.KD_SIM)
    }
  }

  fun periodic() {
    io.updateInputs(inputs)

    val hasPIDChanged: Boolean =
      ClimberTunableValues.kP.hasChanged() ||
        ClimberTunableValues.kI.hasChanged() ||
        ClimberTunableValues.kD.hasChanged()

    val hasFFChanged: Boolean =
      ClimberTunableValues.kS.hasChanged() ||
        ClimberTunableValues.kGDefault.hasChanged() ||
        ClimberTunableValues.kV.hasChanged() ||
        ClimberTunableValues.kA.hasChanged()

    if (hasPIDChanged) {
      io.configClimberPID(
        ClimberTunableValues.kP.get(),
        ClimberTunableValues.kI.get(),
        ClimberTunableValues.kD.get()
      )
    }

    if (hasFFChanged) {
      io.configClimberFF(
        ClimberTunableValues.kGDefault.get(),
        ClimberTunableValues.kS.get(),
        ClimberTunableValues.kV.get(),
        ClimberTunableValues.kA.get()
      )
    }

    CustomLogger.processInputs("Climber", inputs)
    CustomLogger.recordOutput("Climber/currentState", currentState.name)
    CustomLogger.recordOutput("Climber/requestedState", currentRequest.javaClass.simpleName)
    CustomLogger.recordOutput("Climber/climberTargetVoltage", climberTargetVoltage.inVolts)
    CustomLogger.recordOutput("Climber/rollersTargetVoltage", rollersTargetVoltage.inVolts)

    var nextState: ClimberState = currentState

    when (currentState) {
      ClimberState.UNINITIALIZED -> {
        io.zeroEncoder()
        nextState = fromRequestToState(currentRequest)
      }
      ClimberState.OPEN_LOOP -> {
        setClimberVoltage(climberTargetVoltage)
        setRollersVoltage(rollersTargetVoltage)
        nextState = fromRequestToState(currentRequest)
      }
      else -> {}
    }

    currentState = nextState
  }

  private fun setClimberVoltage(voltage: ElectricalPotential) {
    io.setClimberVoltage(voltage)
  }

  private fun setRollersVoltage(voltage: ElectricalPotential) {
    io.setRollersVoltage(voltage)
  }

  companion object {
    enum class ClimberState {
      UNINITIALIZED,
      OPEN_LOOP,
      HOME;

      fun equivalentToRequest(request: Request.ClimberRequest): Boolean {
        return (
          (request is Request.ClimberRequest.Home && this == HOME) ||
            (request is Request.ClimberRequest.OpenLoop && this == OPEN_LOOP)
          )
      }
    }

    fun fromRequestToState(request: Request.ClimberRequest): ClimberState {
      return when (request) {
        is Request.ClimberRequest.Home -> ClimberState.HOME
        is Request.ClimberRequest.OpenLoop -> ClimberState.OPEN_LOOP
      }
    }
  }
}
