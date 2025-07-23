package com.team4099.robot2025.config.constants

import org.team4099.lib.units.LinearAcceleration
import org.team4099.lib.units.LinearVelocity
import org.team4099.lib.units.base.Length
import org.team4099.lib.units.base.Mass
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.inches
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.base.pounds
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.inMetersPerSecond
import org.team4099.lib.units.perSecond

object ElevatorConstants {
  val VOLTAGE_COMPENSATION: ElectricalPotential = 12.0.volts
  val SPOOL_DIAMETER: Length = 2.0.inches
  val GEAR_RATIO: Double = 12.0 / 48.0
  val CARRIAGE_MASS: Mass =
    15.0.pounds // semi-accurate, tbf this value is only used in sim so its whatever

  val STATOR_CURRENT_LIMIT = 60.0.amps
  val SUPPLY_CURRENT_LIMIT = 60.0.amps

  val HOMING_APPLIED_VOLTAGE = -1.0.volts
  val HOMING_STALL_CURRENT = 15.0.amps
  val HOMING_STALL_TIME_THRESHOLD = 0.15.seconds

  val UPWARDS_EXTENSION_LIMIT: Length = 59.375000.inches
  val DOWNWARDS_EXTENSION_LIMIT: Length = 0.inches
  val FIRST_STAGE_HEIGHT: Length = 25.125000.inches

  // TODO: check?
  val MAX_VELOCITY: LinearVelocity = 144.85.inches.perSecond
  val MAX_ACCELERATION: LinearAcceleration = 259.54.inches.perSecond.perSecond

  val ELEVATOR_TOLERANCE = 0.2.inches

  object HEIGHTS {
    val IDLE = 0.0.inches
    val IDLE_CORAL = 0.0.inches
    val IDLE_ALGAE = 0.0.inches

    val INTAKE_CORAL = 0.0.inches
    val INTAKE_ALGAE_GROUND = 0.0.inches
    val INTAKE_ALGAE_LOW = 0.0.inches
    val INTAKE_ALGAE_HIGH = 0.0.inches

    val L1 = 0.0.inches
    val L2 = 0.0.inches
    val L3 = 0.0.inches
    val L4 = 0.0.inches

    val PROCESSOR = 0.0.inches
    val BARGE = 0.0.inches

    val EJECT = 0.0.inches
  }

  object PID {
    // TODO: tune all
    val REAL_KP = 0.0.volts / 1.inches
    val REAL_KI = 0.0.volts / (1.inches * 1.seconds)
    val REAL_KD = 0.0.volts / (1.inches.perSecond)

    val SIM_KP = 2.4.volts / 1.inches
    val SIM_KI = 0.0.volts / (1.inches * 1.seconds)
    val SIM_KD = 0.9.volts / (1.inches.perSecond)

    val KS = 0.0.volts
    val KV = ((1 / MAX_VELOCITY.inMetersPerSecond).volts) / 1.0.meters.perSecond //  0.037
    val KA = (0.0.volts) / 1.0.meters.perSecond.perSecond // 0.0025

    val KV_ADD = (0.0.volts) / 1.0.meters.perSecond //  0.037

    val KG_FIRST_STAGE = 0.0.volts
    val KG_SECOND_STAGE = 0.0.volts
  }
}
