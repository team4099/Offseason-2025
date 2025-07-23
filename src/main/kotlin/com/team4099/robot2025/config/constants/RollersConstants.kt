package com.team4099.robot2025.config.constants

import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.grams
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.meterSquared
import org.team4099.lib.units.derived.volts

object RollersConstants {
  val GEAR_RATIO = 12.0 / (24.0 / 20.0) / (60.0 / 18.0) / (36.0 / 24.0) / 24.0
  val VOLTAGE_COMPENSATION = 12.volts

  val CORAL_CURRENT_THRESHOLD = 0.0.amps
  val CORAL_DETECTION_THRESHOLD = 0.0.seconds
  val ALGAE_CURRENT_THRESHOLD = 0.0.amps
  val ALGAE_DETECTION_THRESHOLD = 0.0.seconds

  val MOMENT_OF_INERTIA = 0.09344594214.grams.meterSquared

  val SUPPLY_CURRENT_LIMIT = 40.0.amps
  val STATOR_CURRENT_LIMIT = 40.0.amps

  val OUTTAKE_VOLTAGE = (-2.0).volts
}
