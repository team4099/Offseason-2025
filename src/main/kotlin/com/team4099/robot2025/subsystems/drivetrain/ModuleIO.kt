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

interface ModuleIO {
  @AutoLog
  class ModuleIOInputs: LoggableInputs {
    var driveConnected: Boolean = false
    var drivePositionRad: Double = 0.0
    var driveVelocityRadPerSec: Double = 0.0
    var driveAppliedVolts: Double = 0.0
    var driveCurrentAmps: Double = 0.0

    var turnConnected: Boolean = false
    var turnEncoderConnected: Boolean = false
    var turnAbsolutePosition: Rotation2d = Rotation2d()
    var turnPosition: Rotation2d = Rotation2d()
    var turnVelocityRadPerSec: Double = 0.0
    var turnAppliedVolts: Double = 0.0
    var turnCurrentAmps: Double = 0.0

    var odometryTimestamps: DoubleArray = doubleArrayOf()
    var odometryDrivePositionsRad: DoubleArray = doubleArrayOf()
    var odometryTurnPositions: Array<Rotation2d> = arrayOf()

    override fun toLog(table: LogTable?) {
      TODO("Not yet implemented")
    }

    override fun fromLog(table: LogTable?) {
      TODO("Not yet implemented")
    }
  }

  /** Updates the set of loggable inputs.  */
  fun updateInputs(inputs: ModuleIOInputs) {}

  /** Run the drive motor at the specified open loop value.  */
  fun setDriveOpenLoop(output: Double) {}

  /** Run the turn motor at the specified open loop value.  */
  fun setTurnOpenLoop(output: Double) {}

  /** Run the drive motor at the specified velocity.  */
  fun setDriveVelocity(velocityRadPerSec: Double) {}

  /** Run the turn motor to the specified rotation.  */
  fun setTurnPosition(rotation: Rotation2d) {}
}
