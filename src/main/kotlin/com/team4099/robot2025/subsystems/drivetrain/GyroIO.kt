// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.
package com.team4099.robot2025.subsystems.drivetrain

import edu.wpi.first.math.geometry.Rotation2d
import org.littletonrobotics.junction.AutoLog
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs

interface GyroIO {
  @AutoLog
  class GyroIOInputs : LoggableInputs {
    @JvmField
    var connected: Boolean = false
    @JvmField
    var yawPosition: Rotation2d = Rotation2d()
    @JvmField
    var yawVelocityRadPerSec: Double = 0.0
    @JvmField
    var odometryYawTimestamps: DoubleArray = doubleArrayOf()
    @JvmField
    var odometryYawPositions: Array<Rotation2d> = arrayOf()

    override fun toLog(table: LogTable?) {
      TODO("Not yet implemented")
    }

    override fun fromLog(table: LogTable?) {
      TODO("Not yet implemented")
    }
  }

  fun updateInputs(inputs: GyroIOInputs) {}
}
