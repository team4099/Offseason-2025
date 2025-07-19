package com.team4099.robot2025.commands

import com.team4099.robot2025.subsystems.elevator.Elevator
import com.team4099.robot2025.subsystems.superstructure.Request
import edu.wpi.first.wpilibj2.command.Command
import org.team4099.lib.units.base.Length

class TestElevatorCommand(val elevator: Elevator, private val height: Length) : Command() {
  init {
    addRequirements(elevator)
  }

  override fun initialize() {
    //        CustomLogger.recordDebugOutput("ActiveCommands/TestElevatorCommand", true)
  }

  override fun execute() {
    elevator.currentRequest = Request.ElevatorRequest.ClosedLoop(height)
    //        if (isFinished) end(false)
  }

  override fun isFinished(): Boolean {
    return elevator.isAtTargetedPosition
  }

  override fun end(interrupted: Boolean) {
    //        CustomLogger.recordDebugOutput("ActiveCommands/TestElevatorCommand", false)
    //        elevator.currentRequest = Request.ElevatorRequest.OpenLoop(0.0.volts)
  }
}
