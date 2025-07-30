package com.team4099.robot2025.subsystems.canRange

import edu.wpi.first.wpilibj2.command.SubsystemBase

class CanRange(val io: CANRangeIO): SubsystemBase() {
    val inputs = CANRangeIO.CANRangeIOInputs()
    var distance = inputs.distance
    var isDetected = inputs.isDetected
    override fun periodic(){
        io.updateInputs(inputs)
    }
}