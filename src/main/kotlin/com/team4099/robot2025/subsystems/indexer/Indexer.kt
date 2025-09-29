package com.team4099.robot2025.subsystems.indexer

import com.team4099.lib.hal.Clock
import com.team4099.robot2025.config.constants.IndexerConstants
import com.team4099.robot2025.subsystems.superstructure.Request
import com.team4099.robot2025.util.CustomLogger
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.team4099.lib.units.base.inSeconds

class Indexer(val io: IndexerIO) : SubsystemBase() {
  val inputs = IndexerIO.IndexerInputs()

  private var lastTransitionTime = Clock.fpgaTime

  val isStuck: Boolean
    get() {
      return inputs.indexerStatorCurrent >= IndexerConstants.CORAL_STALL_CURRENT
    }

  var currentState = IndexerState.UNINITIALIZED
  var currentRequest: Request.IndexerRequest = Request.IndexerRequest.Idle()

  override fun periodic() {
    io.updateInputs(inputs)

    CustomLogger.processInputs("Indexer", inputs)

    CustomLogger.recordOutput("Indexer/currentState", currentState.name)
    CustomLogger.recordOutput("Indexer/currentRequest", currentRequest.javaClass.simpleName)

    CustomLogger.recordOutput("Indexer/isStuck", isStuck)

    var nextState = currentState
    when (currentState) {
      IndexerState.UNINITIALIZED -> {
        nextState = fromRequestToState(currentRequest)
      }
      IndexerState.IDLE -> {
        io.setVoltage(IndexerConstants.IDLE_VOLTAGE)
        nextState = fromRequestToState(currentRequest)
      }
      IndexerState.INDEX -> {
        if ((Clock.fpgaTime - lastTransitionTime).inSeconds % 2 < 1.5)
          io.setVoltage(IndexerConstants.INDEX_VOLTAGE)
        else io.setVoltage(IndexerConstants.SPIT_VOLTAGE)
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
