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

import org.littletonrobotics.junction.AutoLog
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs
import org.team4099.lib.units.AngularVelocity
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.perSecond

interface GyroIO {
  @AutoLog
  class GyroIOInputs : LoggableInputs {
    var connected: Boolean = false
    var yawPosition: Angle = 0.0.radians
    var yawVelocity: AngularVelocity = 0.0.radians.perSecond
    var odometryYawTimestamps: DoubleArray = doubleArrayOf()
    var odometryYawPositions: Array<Angle> = arrayOf()

    override fun toLog(table: LogTable?) {
      TODO("Not yet implemented")
    }

    override fun fromLog(table: LogTable?) {
      TODO("Not yet implemented")
    }
  }

  fun updateInputs(inputs: GyroIOInputs) {}
}
