package com.team4099.robot2025.subsystems.climber

import com.team4099.robot2025.config.constants.ClimberConstants
import com.team4099.robot2025.subsystems.superstructure.Request
import com.team4099.robot2025.util.CustomLogger
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inDegrees
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.volts

class Climber(private val io: ClimberIO) : SubsystemBase() {
  val inputs: ClimberIO.ClimberInputs = ClimberIO.ClimberInputs()

  private var currentState: ClimberState = ClimberState.UNINITIALIZED
  var currentRequest: Request.ClimberRequest = Request.ClimberRequest.Home()
    set(value) {
      when (value) {
        is Request.ClimberRequest.OpenLoop -> {
          climberTargetVoltage = value.climberVoltage
          rollersTargetVoltage = value.rollersVoltage
        }
        is Request.ClimberRequest.ClosedLoop -> {
          climberTargetPosition = value.position
        }
        else -> {}
      }
      field = value
    }

  private var climberTargetVoltage: ElectricalPotential = 0.0.volts
  private var rollersTargetVoltage: ElectricalPotential = 0.0.volts

  private var climberTargetPosition: Angle = 0.0.degrees

  val isFullyClimbed: Boolean
    get() =
      inputs.climberPosition - ClimberConstants.TARGET_TOLERANCE >=
        ClimberConstants.FULLY_CLIMBED_ANGLE

  val isFullyExtended: Boolean
    get() =
      inputs.climberPosition - ClimberConstants.TARGET_TOLERANCE >=
        ClimberConstants.FULLY_EXTENDED_ANGLE

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

    ClimberTunableValues.kS.initDefault(ClimberConstants.PID.KG_DEFAULT)
    ClimberTunableValues.kGDefault.initDefault(ClimberConstants.PID.KG_DEFAULT)
    ClimberTunableValues.kGLatched.initDefault(ClimberConstants.PID.KG_LATCHED)
    ClimberTunableValues.kGUnLatched.initDefault(ClimberConstants.PID.KG_UNLATCHED)
    ClimberTunableValues.kV.initDefault(ClimberConstants.PID.KV)
    ClimberTunableValues.kA.initDefault(ClimberConstants.PID.KA)
  }

  override fun periodic() {
    io.updateInputs(inputs)

    val hasPIDChanged: Boolean =
      ClimberTunableValues.kP.hasChanged() ||
        ClimberTunableValues.kI.hasChanged() ||
        ClimberTunableValues.kD.hasChanged()

    val hasFFChanged: Boolean =
      ClimberTunableValues.kS.hasChanged() ||
        ClimberTunableValues.kGDefault.hasChanged() ||
        ClimberTunableValues.kGLatched.hasChanged() ||
        ClimberTunableValues.kGUnLatched.hasChanged() ||
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
    CustomLogger.recordOutput("Climber/climberTargetPosition", climberTargetPosition.inDegrees)

    CustomLogger.recordOutput("Climber/isFullyClimbed", isFullyClimbed)
    CustomLogger.recordOutput("Climber/isFullyExtended", isFullyExtended)

    var nextState: ClimberState = currentState

    when (currentState) {
      ClimberState.UNINITIALIZED -> {
        nextState = fromRequestToState(currentRequest)
      }
      ClimberState.HOME -> {
        io.zeroEncoder()
        nextState = fromRequestToState(currentRequest)
      }
      ClimberState.OPEN_LOOP -> {
        io.setClimberVoltage(climberTargetVoltage)
        io.setRollersVoltage(rollersTargetVoltage)
        nextState = fromRequestToState(currentRequest)
      }
      ClimberState.CLOSED_LOOP -> {
        io.setClimberPosition(climberTargetPosition)
        nextState = fromRequestToState(currentRequest)
      }
    }

    currentState = nextState
  }

  companion object {
    enum class ClimberState {
      UNINITIALIZED,
      OPEN_LOOP,
      CLOSED_LOOP,
      HOME;

      fun equivalentToRequest(request: Request.ClimberRequest): Boolean {
        return (
          (request is Request.ClimberRequest.Home && this == HOME) ||
            (request is Request.ClimberRequest.OpenLoop && this == OPEN_LOOP) ||
            (request is Request.ClimberRequest.ClosedLoop && this == CLOSED_LOOP)
          )
      }
    }

    fun fromRequestToState(request: Request.ClimberRequest): ClimberState {
      return when (request) {
        is Request.ClimberRequest.Home -> ClimberState.HOME
        is Request.ClimberRequest.OpenLoop -> ClimberState.OPEN_LOOP
        is Request.ClimberRequest.ClosedLoop -> ClimberState.CLOSED_LOOP
      }
    }
  }
}
