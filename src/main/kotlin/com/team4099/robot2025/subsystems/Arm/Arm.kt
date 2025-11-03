package com.team4099.robot2025.subsystems.Arm

import com.team4099.robot2025.config.constants.ArmConstants
import com.team4099.robot2025.subsystems.superstructure.Request
import com.team4099.robot2025.util.CustomLogger
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.ironmaple.simulation.IntakeSimulation
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inDegrees
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.volts

class Arm(val io: ArmIO) : SubsystemBase() {
  val inputs = ArmIO.ArmIOInputs()

  var currentState: ArmState = ArmState.UNINITIALIZED

  var isZeroed = false

  var armTargetVoltage: ElectricalPotential = 0.0.volts

  private var armPositionTarget = 0.0.degrees

  val isAtTargetedPosition: Boolean
    get() =
      (
        currentRequest is Request.ArmRequest.ClosedLoop &&
          (inputs.armPosition - armPositionTarget).absoluteValue <= ArmConstants.ARM_TOLERANCE
        )

  var currentRequest: Request.ArmRequest = Request.ArmRequest.Home()
    set(value) {
      when (value) {
        is Request.ArmRequest.OpenLoop -> {
          armTargetVoltage = value.armVoltage
        }
        is Request.ArmRequest.ClosedLoop -> {
          armPositionTarget = value.armPosition
        }
        else -> {}
      }
      field = value
    }

  var isHomed = false

  val algaeIntakeSimulation: IntakeSimulation?
    get() = io.intakeSimulation

  init {
    io.configFF(ArmConstants.PID.KG, ArmConstants.PID.KS, ArmConstants.PID.KV, ArmConstants.PID.KA)

    if (RobotBase.isReal()) {
      io.configPID(
        ArmConstants.PID.REAL_KP,
        ArmConstants.PID.REAL_KI,
        ArmConstants.PID.REAL_KD,
      )
    } else {
      io.configPID(ArmConstants.PID.SIM_KP, ArmConstants.PID.SIM_KI, ArmConstants.PID.SIM_KD)
    }
  }

  override fun periodic() {
    io.updateInputs(inputs)

    CustomLogger.processInputs("Arm", inputs)

    CustomLogger.recordOutput("Arm/currentState", currentState.name)

    CustomLogger.recordOutput("Arm/currentRequest", currentRequest.javaClass.simpleName)
    CustomLogger.recordOutput("Arm/isAtTargetPosition", isAtTargetedPosition)

    CustomLogger.recordOutput("Arm/targetVoltage", armTargetVoltage.inVolts)
    CustomLogger.recordOutput("Arm/targetPosition", armPositionTarget.inDegrees)

    //    if (ArmTunableValues.armkP.hasChanged() ||
    //      ArmTunableValues.armkI.hasChanged() ||
    //      ArmTunableValues.armkD.hasChanged()
    //    ) {
    //      io.configPID(
    //        ArmTunableValues.armkP.get(), ArmTunableValues.armkI.get(),
    // ArmTunableValues.armkD.get()
    //      )
    //    }
    //
    //    if (ArmTunableValues.armkS.hasChanged() ||
    //      ArmTunableValues.armkV.hasChanged() ||
    //      ArmTunableValues.armkA.hasChanged() ||
    //      ArmTunableValues.armkG.hasChanged()
    //    ) {
    //      io.configFF(
    //        ArmTunableValues.armkG.get(),
    //        ArmTunableValues.armkS.get(),
    //        ArmTunableValues.armkV.get(),
    //        ArmTunableValues.armkA.get()
    //      )
    //    }

    var nextState = currentState
    when (currentState) {
      ArmState.UNINITIALIZED -> {
        nextState = fromRequestToState(currentRequest)
      }
      ArmState.HOME -> {
        io.zeroEncoder()
        currentRequest = Request.ArmRequest.OpenLoop(0.volts)
        isZeroed = true
        nextState = fromRequestToState(currentRequest)
      }
      ArmState.OPEN_LOOP -> {
        io.setVoltage(armTargetVoltage)
        nextState = fromRequestToState(currentRequest)
      }
      ArmState.TARGETING_POS -> {
        io.setPosition(armPositionTarget)
        nextState = fromRequestToState(currentRequest)
      }
    }
    currentState = nextState
  }

  companion object {
    enum class ArmState {
      UNINITIALIZED,
      HOME,
      OPEN_LOOP,
      TARGETING_POS
    }
    inline fun fromRequestToState(request: Request.ArmRequest): ArmState {
      return when (request) {
        is Request.ArmRequest.OpenLoop -> ArmState.OPEN_LOOP
        is Request.ArmRequest.ClosedLoop -> ArmState.TARGETING_POS
        is Request.ArmRequest.Home -> ArmState.HOME
      }
    }
  }
}
