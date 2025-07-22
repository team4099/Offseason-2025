package com.team4099.robot2025.config.constants

import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.inches
import org.team4099.lib.units.base.pounds
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.DerivativeGain
import org.team4099.lib.units.derived.IntegralGain
import org.team4099.lib.units.derived.ProportionalGain
import org.team4099.lib.units.derived.Radian
import org.team4099.lib.units.derived.Volt
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.perSecond

object IntakeConstants {
    val ZERO_OFFSET = 0.0.degrees
    val PIVOT_GEAR_RATIO = 20.0 / 1.0

    val PIVOT_INERTIA = 17.586.pounds * 1.0.inches.squared
    val PIVOT_LENGTH = 13.51441.inches
    val PIVOT_MAX_ANGLE = 145.0.degrees
    val PIVOT_MIN_ANGLE = 0.0.degrees

    // Feedforward Constants
    val PIVOT_KA = 0.0.volts / 1.0.radians.perSecond.perSecond
    val PIVOT_KV = 0.0.volts / 1.0.radians.perSecond
    val PIVOT_KG = 0.15.volts
    val PIVOT_KS = 0.0.volts

    val STATOR_CURRENT_LIMIT = 60.amps
    val SUPPLY_CURRENT_LIMIT = 60.amps

    val VOLTAGE_COMPENSATION = 12.0.volts

    val MAX_VELOCITY = 400.degrees.perSecond
    val MAX_ACCELERATION = 400.degrees.perSecond.perSecond

    val SIM_VELOCITY = 400.degrees.perSecond
    val SIM_ACCELERATION = 400.degrees.perSecond.perSecond

    object PID {
        // PID Constants
        val SIM_PIVOT_KP: ProportionalGain<Radian, Volt> = 0.15.volts / 1.0.degrees
        val SIM_PIVOT_KI: IntegralGain<Radian, Volt> = 0.01.volts / (1.0.degrees * 1.0.seconds)
        val SIM_PIVOT_KD: DerivativeGain<Radian, Volt> = 0.004.volts / 1.0.degrees.perSecond

        val REAL_PIVOT_KP: ProportionalGain<Radian, Volt> = 7.volts / 1.0.degrees
        val REAL_PIVOT_KI: IntegralGain<Radian, Volt> = 0.0.volts / (1.0.degrees * 1.0.seconds)
        val REAL_PIVOT_KD: DerivativeGain<Radian, Volt> = 0.3.volts / 1.0.degrees.perSecond
    }

    object Rollers {
        val GEAR_RATIO = 1 / 1.0
        val INERTIA = 5.141e-13.pounds * 1.0.inches.squared

        val VOLTAGE_COMPENSATION = 12.volts

        val STATOR_CURRENT_LIMIT = 40.amps
        val SUPPLY_CURRENT_LIMIT = 40.amps
    }
}