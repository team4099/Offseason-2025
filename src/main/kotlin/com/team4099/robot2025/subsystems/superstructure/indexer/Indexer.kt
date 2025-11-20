package com.team4099.robot2025.subsystems.superstructure.indexer

import com.team4099.lib.hal.Clock
import com.team4099.robot2025.config.constants.IndexerConstants
import com.team4099.robot2025.subsystems.superstructure.Request
import com.team4099.robot2025.util.ControlledByStateMachine
import com.team4099.robot2025.util.CustomLogger
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.volts

class Indexer(val io: IndexerIO) : ControlledByStateMachine() {
  val inputs = IndexerIO.IndexerInputs()

  private var lastTransitionTime = Clock.fpgaTime

  val hasCoral: Boolean
    get() {
      return inputs.indexerStatorCurrent >= IndexerConstants.CORAL_CURRENT
    }
  var lastCoralTriggerTime = 0.seconds
  var targetVoltage = 0.0.volts

  var currentState = IndexerState.UNINITIALIZED
  var currentRequest: Request.IndexerRequest = Request.IndexerRequest.Idle()
    set(value) {
      if (value is Request.IndexerRequest.Index) {
        targetVoltage = value.voltage
      }
      field = value
    }

  override fun loop() {
    io.updateInputs(inputs)

    CustomLogger.processInputs("Indexer", inputs)

    CustomLogger.recordOutput("Indexer/currentState", currentState.name)
    CustomLogger.recordOutput("Indexer/currentRequest", currentRequest.javaClass.simpleName)

    var nextState = currentState
    when (currentState) {
      IndexerState.UNINITIALIZED -> {
        nextState = fromRequestToState(currentRequest)
      }
      IndexerState.IDLE -> {
        io.setVoltage(IndexerConstants.IDLE_VOLTAGE)
        lastCoralTriggerTime = 0.seconds
        nextState = fromRequestToState(currentRequest)
      }
      IndexerState.INDEX -> {
        if (hasCoral && lastCoralTriggerTime == 0.seconds) lastCoralTriggerTime = Clock.fpgaTime
        io.setVoltage(targetVoltage)
        nextState = fromRequestToState(currentRequest)
      }
      IndexerState.EJECT -> {
        io.setVoltage(IndexerConstants.SPIT_VOLTAGE)
        nextState = fromRequestToState(currentRequest)
      }
    }

    if (nextState != currentState) lastTransitionTime = Clock.fpgaTime

    currentState = nextState
  }

  companion object {
    enum class IndexerState {
      UNINITIALIZED,
      IDLE,
      INDEX,
      EJECT
    }
    fun fromRequestToState(request: Request.IndexerRequest): IndexerState {
      return when (request) {
        is Request.IndexerRequest.Idle -> IndexerState.IDLE
        is Request.IndexerRequest.Index -> IndexerState.INDEX
        is Request.IndexerRequest.Eject -> IndexerState.EJECT
      }
    }
  }
}
