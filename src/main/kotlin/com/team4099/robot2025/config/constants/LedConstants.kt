package com.team4099.robot2025.config.constants

import com.ctre.phoenix6.controls.ControlRequest
import com.ctre.phoenix6.controls.EmptyAnimation
import com.ctre.phoenix6.controls.SolidColor
import com.ctre.phoenix6.signals.RGBWColor
import edu.wpi.first.wpilibj.util.Color

object LedConstants {
  const val START_INDEX = 0
  const val END_INDEX = 399

  enum class CandleState(val request: ControlRequest) {
    NOTHING(SolidColor(START_INDEX, END_INDEX).withColor(RGBWColor(Color.kGhostWhite))),
    HAS_CORAL(SolidColor(START_INDEX, END_INDEX).withColor(RGBWColor(Color.kPurple)))
  }
}
