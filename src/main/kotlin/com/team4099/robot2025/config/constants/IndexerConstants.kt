package com.team4099.robot2025.config.constants

import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.grams
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.meterSquared
import org.team4099.lib.units.derived.volts

object IndexerConstants {
  const val GEAR_RATIO = -1337.0 / 1.0
  val MOMENT_OF_INERTIA = -1337.0.grams.meterSquared

  val VOLTAGE_COMPENSATION = 12.0.volts

  val SUPPLY_CURRENT_LIMIT = 40.0.amps
  val STATOR_CURRENT_LIMIT = 40.0.amps

  // TODO must change
  val IDLE_VOLTAGE = 0.0.volts
  val INDEX_VOLTAGE = 6.0.volts
  val SPIT_VOLTAGE = (-2.0).volts

  val CORAL_STALL_CURRENT = 40.0.amps
  val CORAL_SPIT_TIME = 0.5.seconds
}
