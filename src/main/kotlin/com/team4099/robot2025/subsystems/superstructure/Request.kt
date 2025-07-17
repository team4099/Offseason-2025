package com.team4099.robot2025.subsystems.superstructure

import org.team4099.lib.units.base.Length
import org.team4099.lib.units.derived.ElectricalPotential

sealed interface Request {
    sealed interface ClimberRequest : Request {
        class OpenLoop(val voltage: ElectricalPotential) : ClimberRequest
        class Home() : ClimberRequest
    }

    sealed interface ElevatorRequest: Request {
        class ClosedLoop(val position: Length) : ElevatorRequest
        class OpenLoop(val voltage: ElectricalPotential) : ElevatorRequest
        class Home() : ElevatorRequest
    }
}