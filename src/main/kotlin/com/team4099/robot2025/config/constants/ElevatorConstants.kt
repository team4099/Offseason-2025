package com.team4099.robot2025.config.constants

import org.team4099.lib.units.LinearAcceleration
import org.team4099.lib.units.LinearVelocity
import org.team4099.lib.units.base.*
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.perSecond

object ElevatorConstants { //TODO add ts coonstants @magigoon
    val VOLTAGE_COMPENSATION: ElectricalPotential = 12.0.volts
    val SPOOL_DIAMETER: Length = 3.5.inches
    val GEAR_RATIO: Double = 72.0 / 12.0
    val FIRST_STAGE_HEIGHT: Length = 0.0.inches

    val LEADER_STATOR_CURRENT_LIMIT = 60.0.amps
    val LEADER_SUPPLY_CURRENT_LIMIT = 60.0.amps
    val LEADER_SUPPLY_CURRENT_LOWER_LIMIT = 60.0.amps
    val LEADER_SUPPLY_CURRENT_LOWER_TIME = 60.0.amps

    val FOLLOWER_STATOR_CURRENT_LIMIT = 60.0.amps
    val FOLLOWER_SUPPLY_CURRENT_LIMIT = 60.0.amps
    val FOLLOWER_SUPPLY_CURRENT_LOWER_LIMIT = 60.0.amps
    val FOLLOWER_SUPPLY_CURRENT_LOWER_TIME = 60.0.amps

    val UPWARDS_EXTENSION_LIMIT: Length = 0.inches
    val DOWNWARDS_EXTENSION_LIMIT: Length = 0.inches

    val MAX_VELOCITY: LinearVelocity = 0.meters.perSecond
    val MAX_ACCELERATION: LinearAcceleration = 0.meters.perSecond.perSecond

    object PID {
        // TODO: tune all
        val REAL_KP = 0.0.volts / 1.inches
        val REAL_KI = 0.0.volts / (1.inches * 1.seconds)
        val REAL_KD = 0.0.volts / (1.inches.perSecond)

        val SIM_KP = 0.0.volts / 1.inches
        val SIM_KI = 0.0.volts / (1.inches * 1.seconds)

        val SIM_KD = 0.0.volts / (1.inches.perSecond)

        val KS = 0.0.volts
        val KV = (0.0.volts) / 1.0.meters.perSecond //  0.037
        val KA = (0.0.volts) / 1.0.meters.perSecond.perSecond // 0.0025

        val KV_ADD = (0.0.volts) / 1.0.meters.perSecond //  0.037

        val KG_DEFAULT = 0.0.volts
        val KG_FIRST_STAGE = 0.0.volts
        val KG_SECOND_STAGE = 0.0.volts
    }
}