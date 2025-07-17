package com.team4099.robot2025.subsystems.superstructure

import org.team4099.lib.units.derived.ElectricalPotential

sealed interface Request {
    sealed interface ClimberRequest : Request {
        class OpenLoop(val voltage: ElectricalPotential) : ClimberRequest
        class Home() : ClimberRequest
    }
}