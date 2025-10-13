package com.team4099.robot2025.subsystems.led

import com.ctre.phoenix6.configs.CANdleConfiguration
import com.ctre.phoenix6.controls.EmptyAnimation
import com.ctre.phoenix6.controls.SolidColor
import com.ctre.phoenix6.hardware.CANdle
import com.ctre.phoenix6.signals.LossOfSignalBehaviorValue
import com.ctre.phoenix6.signals.RGBWColor
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
    val color = RGBWColor(state.r, state.g, state.b)

    val request =
      when (state.animation) {
        null -> SolidColor(LedConstants.START_INDEX, LedConstants.END_INDEX)
        // add more animations when needed
        else -> EmptyAnimation(0)
      }
  }
}
