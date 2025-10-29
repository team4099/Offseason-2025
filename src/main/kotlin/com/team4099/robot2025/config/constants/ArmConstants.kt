package com.team4099.robot2025.config.constants

import org.team4099.lib.units.base.grams
import org.team4099.lib.units.base.inches
import org.team4099.lib.units.base.pounds
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.meterSquared
import org.team4099.lib.units.derived.rotations
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.kilo
import org.team4099.lib.units.perSecond

object ArmConstants {
  val VOLTAGE_COMPENSATION = 12.0.volts
  // what armPosition is when the arm is 0 in +x direction (above intake)
  val ENCODER_ANGLE_OFFSET = 0.187.rotations

  val ARM_TOLERANCE = 2.5.degrees
  val ARM_LENGTH = 20.inches
  val ARM_LENGTH_TO_ALGAE_CENTER = 18.inches + 16.inches / 2.0
  val ARM_MOMENT_OF_INERTIA = 0.00503098.kilo.grams.meterSquared
  val ARM_MASS = 4.5796764.pounds

  val MIN_ROTATION = -325.degrees
  val MAX_ROTATION = 45.degrees

  val GEAR_RATIO = (14.0 / 62.0) * (26.0 / 54.0) * (12.0 / 66.0)
  val ENCODER_TO_MECHANISM_GEAR_RATIO = 1.0

  val CANCODER_DISCONTINUITY_POINT = 0.0.rotations

  val MAX_VELOCITY = 300.0
  val MAX_ACCELERATION = 250.0

  val SIM_INTAKE_WIDTH = 6.5.inches
  val SIM_INTAKE_HALFHEIGHT = 4.inches

  val TIME_TO_GOAL = 0.5.seconds

  object PID {
    val REAL_KP = 1.5.volts / 1.degrees
    val REAL_KI = 0.0.volts / (1.degrees * 1.seconds)
    val REAL_KD = 0.05.volts / (1.degrees.perSecond)

    val SIM_KP = 0.75.volts / 1.degrees
    val SIM_KI = 0.0.volts / (1.degrees * 1.seconds)
    val SIM_KD = 0.02.volts / (1.degrees.perSecond)

    val KS = 0.2.volts
    val KV = ((0.0 / MAX_VELOCITY).volts) / 1.0.degrees.perSecond //  0.037
    val KA = (0.0.volts) / 1.0.degrees.perSecond.perSecond // 0.0025

    val KG = 0.15.volts
  }

  // all angles were originally measured from cw down, so the -(value + 90) adjusts it properly to
  // be ccw horizontal
  object ANGLES {
    val IDLE_ANGLE = -(0.0.degrees + 90.degrees)
    val EJECT_ANGLE = -(60.0.degrees + 90.degrees)
    val HOME_ANGLE = -(180.0.degrees + 90.degrees)
    val CLIMB_ANGLE = -(90.0.degrees + 90.degrees)

    val INTAKE_CORAL_ANGLE = -(0.0.degrees + 90.degrees)

    val IDLE_CORAL_ANGLE = -(0.0.degrees + 90.degrees)
    val IDLE_ALGAE_ANGLE = -(155.0.degrees + 90.degrees)

    val L1_PREP_ANGLE = -(70.0.degrees + 90.degrees)
    val L2_PREP_ANGLE = -(123.26.degrees + 90.degrees)
    val L3_PREP_ANGLE = -(123.26.degrees + 90.degrees)
    val L4_PREP_ANGLE = -(127.68.degrees + 90.degrees)

    val BARGE_ANGLE = -225.degrees
    val PROCESSOR_ANGLE = -(70.0.degrees + 90.degrees)
    val BARGE_POST_SHOOT_ANGLE = -(270.degrees + 30.degrees)
    val BARGE_SHOOT_THRESHOLD = -(270.degrees - 30.degrees)

    val ALGAE_GROUND_INTAKE_ANGLE = -(79.62.degrees + 90.degrees)
    val ALGAE_LOW_INTAKE_ANGLE = -(90.0.degrees + 90.degrees)
    val ALGAE_HIGH_INTAKE_ANGLE = -(142.degrees)

    val SCORE_ANGLE_OFFSET = -40.degrees

    val ARM_GUARENTEED_OVER_BATTERY = -(80.0.degrees + 90.degrees)

    val SIM_MECH_OFFSET = 90.0.degrees

    val MOVING_BETWEEN_REEF_LEVELS_ANGLE = -(170.0.degrees + 90.degrees)

    val MANUAL_ZERO_ANGLE = -270.degrees
  }
}
