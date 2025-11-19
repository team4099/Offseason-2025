package com.team4099.robot2025.subsystems.superstructure.climber

import com.team4099.lib.hal.Clock
import com.team4099.robot2025.config.constants.ClimberConstants
import com.team4099.robot2025.subsystems.superstructure.Request
import com.team4099.robot2025.util.ControlledByStateMachine
import com.team4099.robot2025.util.CustomLogger
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.team4099.lib.units.base.inMilliseconds
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inDegrees
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.volts

class Climber(private val io: ClimberIO) : ControlledByStateMachine() {
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
      inputs.climberPosition + ClimberConstants.TARGET_TOLERANCE >=
        ClimberConstants.FULLY_CLIMBED_ANGLE

  val isFullyExtended: Boolean
    get() =
      inputs.climberPosition + ClimberConstants.TARGET_TOLERANCE >=
        ClimberConstants.FULLY_EXTENDED_ANGLE

  init {
    if (RobotBase.isReal()) {
      io.configClimberPID(
        ClimberConstants.PID.KP_REAL, ClimberConstants.PID.KI_REAL, ClimberConstants.PID.KD_REAL
      )
    } else {
      io.configClimberPID(
        ClimberConstants.PID.KP_SIM, ClimberConstants.PID.KI_SIM, ClimberConstants.PID.KD_SIM
      )
    }

    io.configClimberFF(
      ClimberConstants.PID.KG_DEFAULT,
      ClimberConstants.PID.KS,
      ClimberConstants.PID.KV,
      ClimberConstants.PID.KA
    )
  }

  override fun loop() {
    val startTime = Clock.fpgaTime

    io.updateInputs(inputs)

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

    CustomLogger.recordOutput(
      "LoggedRobot/Subsystems/ClimberLoopTimeMS", (Clock.fpgaTime - startTime).inMilliseconds
    )
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
