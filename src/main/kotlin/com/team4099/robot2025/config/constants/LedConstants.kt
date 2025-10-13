package com.team4099.robot2025.config.constants

import com.ctre.phoenix.led.Animation

object LedConstants {

  const val START_INDEX = 8
  const val END_INDEX = 399

  enum class CandleState(val animation: Animation?, val r: Int, val g: Int, val b: Int) {
    NOTHING(null, 0, 0, 0),
    HAS_CORAL(null, 255, 0, 0)
  }
}
