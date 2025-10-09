package com.team4099.robot2025.subsystems.led

import com.ctre.phoenix6.configs.CANdleConfiguration
import com.ctre.phoenix6.hardware.CANdle
import com.ctre.phoenix6.signals.LossOfSignalBehaviorValue
import com.team4099.robot2025.config.constants.Constants
import com.team4099.robot2025.config.constants.LedConstants

object LedIOCandle : LedIO {
  private val candle = CANdle(Constants.Candle.CANDLE_ID)
  private val configs = CANdleConfiguration()

  init {
    candle.clearStickyFaults()

    configs.LED.LossOfSignalBehavior = LossOfSignalBehaviorValue.DisableLEDs

    candle.configurator.apply(configs)
  }

  override fun setState(state: LedConstants.CandleState) {
    candle.setControl(state.request)
  }
}
