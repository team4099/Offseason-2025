package com.team4099.robot2025.config.constants

import org.team4099.lib.units.AngularAcceleration
import org.team4099.lib.units.AngularVelocity
import org.team4099.lib.units.base.grams
import org.team4099.lib.units.base.inches
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.meterSquared
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.inDegreesPerSecond
import org.team4099.lib.units.kilo
import org.team4099.lib.units.perSecond

object ArmConstants {
  val VOLTAGE_COMPENSATION = 12.0.volts
  val ENCODER_TO_MECHANISM_GEAR_RATIO = 1.0 / 2.0

  val ARM_TOLERANCE = 1.0.degrees
  val ARM_LENGTH = 26.1362839.inches
  val ARM_MOMENT_OF_INERTIA = 0.01939647.kilo.grams.meterSquared

  val MIN_ROTATION = 0.0.degrees
  val MAX_ROTATION = 270.0.degrees

  val GEAR_RATIO = (14.0 / 62.0) * (26.0 / 54.0)

  val MAX_VELOCITY: AngularVelocity = 270.0.degrees.perSecond
  val MAX_ACCELERATION: AngularAcceleration = 270.0.degrees.perSecond.perSecond

  object PID {
    val REAL_KP = 0.0.volts / 1.degrees
    val REAL_KI = 0.0.volts / (1.degrees * 1.seconds)
    val REAL_KD = 0.0.volts / (1.degrees.perSecond)

    val SIM_KP = 2.4.volts / 1.degrees
    val SIM_KI = 0.0.volts / (1.degrees * 1.seconds)
    val SIM_KD = 0.9.volts / (1.degrees.perSecond)

    val KS = 0.0.volts
    val KV = ((1 / MAX_VELOCITY.inDegreesPerSecond).volts) / 1.0.degrees.perSecond //  0.037
    val KA = (0.0.volts) / 1.0.degrees.perSecond.perSecond // 0.0025

    val KV_ADD = (0.0.volts) / 1.0.degrees.perSecond //  0.037

    val KG = 0.0.volts
  }

  object ANGLES {
    val IDLE_ANGLE = 0.0.degrees
    val EJECT_ANGLE = 0.0.degrees
    val HOME_ANGLE = 0.0.degrees
    val CLIMB_ANGLE = 0.0.degrees

    val INTAKE_CORAL_ANGLE = 0.0.degrees

    val IDLE_CORAL_ANGLE = 0.0.degrees
    val IDLE_ALGAE_ANGLE = 180.0.degrees

    val L1_PREP_ANGLE = 74.0.degrees
    val L2_PREP_ANGLE = 123.26.degrees
    val L3_PREP_ANGLE = 123.26.degrees
    val L4_PREP_ANGLE = 127.68.degrees

    val BARGE_ANGLE = 152.17.degrees
    val PROCESSOR_ANGLE = 70.0.degrees

    val ALGAE_GROUND_INTAKE_ANGLE = 79.62.degrees
    val ALGAE_LOW_INTAKE_ANGLE = 90.0.degrees
    val ALGAE_HIGH_INTAKE_ANGLE = 90.0.degrees

    val SCORE_ANGLE_OFFSET = 30.0.degrees

    val ARM_GUARENTEED_OVER_BATTERY = 45.0.degrees
  }
}
