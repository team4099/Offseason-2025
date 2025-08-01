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

  val IDLE_VOLTAGE = 0.0.volts
  val EJECT_VOLTAGE =0.0.volts

  val IDLE_CORAL_VOLTAGE = 0.volts
  val IDLE_ALGAE_VOLTAGE = 0.5.volts

  val MOMENT_OF_INERTIA = 0.09344594214.grams.meterSquared

  val SUPPLY_CURRENT_LIMIT = 40.0.amps
  val STATOR_CURRENT_LIMIT = 40.0.amps

  // TODO: Set intake voltage values
  val INTAKE_CORAL_VOLTAGE = 1.0.volts
  val INTAKE_ALGAE_VOLTAGE = 1.0.volts

  val OUTTAKE_CORAL_VOLTAGE = (-4.0).volts
  val OUTTAKE_ALGAE_VOLTAGE = (-8.0).volts

  val GAMEPIECE_SPITOUT_THRESHOLD = 1.0.seconds
}
