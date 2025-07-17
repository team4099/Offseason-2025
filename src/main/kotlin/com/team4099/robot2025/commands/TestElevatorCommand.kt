package com.team4099.robot2025.commands

import com.team4099.robot2025.subsystems.elevator.Elevator
import com.team4099.robot2025.subsystems.superstructure.Request
import com.team4099.robot2025.util.CustomLogger
import edu.wpi.first.wpilibj2.command.Command
import org.team4099.lib.units.base.Length
import org.team4099.lib.units.derived.volts

class TestElevatorCommand(val elevator: Elevator, private val height: Length): Command() {
    init {
        addRequirements(elevator)
    }

    override fun initialize() {
        elevator.currentRequest = Request.ElevatorRequest.ClosedLoop(height)
    }

    override fun execute() {
        CustomLogger.recordDebugOutput("ActiveCommands/TestElevatorCommand", true)
        if (elevator.isAtTargetedPosition) end(false)
    }

    override fun isFinished(): Boolean {
        return elevator.isAtTargetedPosition
    }

    override fun end(interrupted: Boolean) {
        elevator.currentRequest = Request.ElevatorRequest.OpenLoop(0.0.volts)
    }
}