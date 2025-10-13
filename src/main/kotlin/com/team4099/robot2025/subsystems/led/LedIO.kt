package com.team4099.robot2025.subsystems.led

import com.team4099.robot2025.config.constants.LedConstants
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs

interface LedIO {
  class LEDIOInputs : LoggableInputs {
    override fun toLog(table: LogTable?) {}
    override fun fromLog(table: LogTable?) {}
  }

  fun setState(state: LedConstants.CandleState) {}
  fun turnOff() {}
}
