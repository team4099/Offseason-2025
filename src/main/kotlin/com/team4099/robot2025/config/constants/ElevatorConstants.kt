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
  val CARRIAGE_MASS: Mass = 7.2911706.pounds + ArmConstants.ARM_MASS

  val STATOR_CURRENT_LIMIT = 40.0.amps
  val SUPPLY_CURRENT_LIMIT = 40.0.amps

  val HOMING_APPLIED_VOLTAGE = (-5.0).volts
  val HOMING_STALL_CURRENT = 15.0.amps
  val HOMING_STALL_TIME_THRESHOLD = 0.35.seconds

  val UPWARDS_EXTENSION_LIMIT: Length = 59.25.inches
  val DOWNWARDS_EXTENSION_LIMIT: Length = 0.inches
  val FIRST_STAGE_HEIGHT: Length = 26.125.inches

  // TODO: check?
  val MAX_VELOCITY: LinearVelocity = 200.inches.perSecond
  val MAX_ACCELERATION: LinearAcceleration = 250.inches.perSecond.perSecond

  val ELEVATOR_TOLERANCE = 0.25.inches

  val CARRIAGE_TO_BOTTOM = 11.0.inches
  val CARRIAGE_TO_BOTTOM_SIM = 0.3.meters // idk why its diff but this works better for as

  object HEIGHTS {
    // note(nathan): make the best attempt to keep IDLE and IDLE_CORAL the same.
    // IDLE and IDLE_CORAL should be high enough that arm CLEARS TROUGH during movement ( > 21 in)
    val IDLE = 22.42.inches
    val IDLE_CORAL = 22.42.inches
    val IDLE_ALGAE = 11.9.inches
    val CLIMB_HEIGHT = 25.0.inches

    val INTAKE_CORAL = 16.25.inches // todo remeasure
    val INTAKE_ALGAE_GROUND = 11.0.inches - CARRIAGE_TO_BOTTOM
    val INTAKE_ALGAE_LOW = 34.56.inches - CARRIAGE_TO_BOTTOM
    val INTAKE_ALGAE_HIGH = 50.46.inches - CARRIAGE_TO_BOTTOM

    val L1 = 28.25.inches - CARRIAGE_TO_BOTTOM - 0.5.inches
    val L2 = 24.4.inches - CARRIAGE_TO_BOTTOM
    val L3 = 40.25.inches - CARRIAGE_TO_BOTTOM
    val L4 = 63.03.inches - CARRIAGE_TO_BOTTOM

    // prevent clipping rising up to L1
    val L1_INIT = L1 + 4.0.inches

    val ZERO_TO_HOME_THRESHOLD = 0.0.inches

    val PROCESSOR = 24.4.inches - CARRIAGE_TO_BOTTOM
    val BARGE = UPWARDS_EXTENSION_LIMIT - 0.25.inches

    // this is to make sure arm doesn't hit battery
    // note(nathan): the following should always be true statements (please please please don't
    // change 👍👍👍👍👍)
    // CLEARS_ROBOT < IDLE
    // CLEARS_ROBOT < IDLE_CORAL
    val CLEARS_ROBOT =
      18.0.inches // todo update with final robot to make sure nothing breaks !!!!!!

    val EJECT = IDLE_CORAL

    val ARM_IDLE_PRIORITY_THRESHOLD = 25.inches
  }

  object PID {
    // TODO: tune all
    val REAL_KP = 2.5.volts / 1.inches
    val REAL_KI = 0.0.volts / (1.inches * 1.seconds)
    val REAL_KD = 0.3.volts / (1.inches.perSecond)

    val SIM_KP = 1.95.volts / 1.inches
    val SIM_KI = 0.0.volts / (1.inches * 1.seconds)
    val SIM_KD = 0.23.volts / (1.inches.perSecond)

    val KS = 0.05.volts
    val KV = ((.0 / MAX_VELOCITY.inMetersPerSecond).volts) / 1.0.meters.perSecond //  0.037
    val KA = (0.0.volts) / 1.0.meters.perSecond.perSecond // 0.0025

    val KV_ADD = (0.0.volts) / 1.0.meters.perSecond //  0.037

    val KG_SIM = 0.3.volts
    val KG_FIRST_STAGE = 0.2.volts
    val KG_SECOND_STAGE = 0.3.volts
  }
}
