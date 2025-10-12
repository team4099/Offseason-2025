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
import org.team4099.lib.units.AngularVelocity
import org.team4099.lib.units.base.Current
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.perSecond

interface ModuleIO {
  @AutoLog
  class ModuleIOInputs: LoggableInputs {
    var driveConnected: Boolean = false
    var drivePosition: Angle = 0.0.radians
    var driveVelocity: AngularVelocity = 0.0.radians.perSecond
    var driveAppliedVoltage: ElectricalPotential = 0.0.volts
    var driveCurrent: Current = 0.0.amps

    var turnConnected: Boolean = false
    var turnEncoderConnected: Boolean = false
    var turnAbsolutePosition: Angle = 0.0.radians
    var turnPosition: Angle = 0.0.radians
    var turnVelocity: AngularVelocity = 0.0.radians.perSecond
    var turnAppliedVoltage: ElectricalPotential = 0.0.volts
    var turnCurrent: Current = 0.0.amps

    var odometryTimestamps: DoubleArray = doubleArrayOf()
    var odometryDrivePositions: Array<Angle> = arrayOf()
    var odometryTurnPositions: Array<Angle> = arrayOf()

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
