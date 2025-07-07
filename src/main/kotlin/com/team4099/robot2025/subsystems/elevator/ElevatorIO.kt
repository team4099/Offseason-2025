package com.team4099.robot2025.subsystems.elevator

import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs

interface ElevatorIO {
    class ElevatorInputs : LoggableInputs {

        override fun toLog(table: LogTable?) {}

        override fun fromLog(table: LogTable?) {}
    }
}