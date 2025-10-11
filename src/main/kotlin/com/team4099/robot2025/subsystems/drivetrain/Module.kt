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

import com.ctre.phoenix6.configs.CANcoderConfiguration
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.swerve.SwerveModuleConstants
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.Alert
import org.littletonrobotics.junction.Logger

class Module(
  private val io: ModuleIO,
  private val index: Int,
  private val constants: SwerveModuleConstants<TalonFXConfiguration?, TalonFXConfiguration?, CANcoderConfiguration?>
) {
  private val inputs: ModuleIO.ModuleIOInputs = ModuleIO.ModuleIOInputs()


  private val driveDisconnectedAlert = Alert(
    "Disconnected drive motor on module $index.",
    Alert.AlertType.kError
  )
  private val turnDisconnectedAlert = Alert(
    "Disconnected turn motor on module $index.", Alert.AlertType.kError
  )
  private val turnEncoderDisconnectedAlert = Alert(
    "Disconnected turn encoder on module $index.",
    Alert.AlertType.kError
  )

  /** Returns the module positions received this cycle.  */
  var odometryPositions: Array<SwerveModulePosition?> = arrayOf()
    private set

  fun periodic() {
    io.updateInputs(inputs)
    Logger.processInputs("Drive/Module$index", inputs)

    // Calculate positions for odometry
    val sampleCount: Int = inputs.odometryTimestamps.size // All signals are sampled together
    odometryPositions = arrayOfNulls(sampleCount)
    for (i in 0 until sampleCount) {
      val positionMeters: Double = inputs.odometryDrivePositionsRad[i] * constants.WheelRadius
      val angle: Rotation2d = inputs.odometryTurnPositions[i]
      odometryPositions[i] = SwerveModulePosition(positionMeters, angle)
    }

    // Update alerts
    driveDisconnectedAlert.set(!inputs.driveConnected)
    turnDisconnectedAlert.set(!inputs.turnConnected)
    turnEncoderDisconnectedAlert.set(!inputs.turnEncoderConnected)
  }

  /** Runs the module with the specified setpoint state. Mutates the state to optimize it.  */
  fun runSetpoint(state: SwerveModuleState) {
    // Optimize velocity setpoint
    state.optimize(angle)
    state.cosineScale(inputs.turnPosition)

    // Apply setpoints
    io.setDriveVelocity(state.speedMetersPerSecond / constants.WheelRadius)
    io.setTurnPosition(state.angle)
  }

  /** Runs the module with the specified output while controlling to zero degrees.  */
  fun runCharacterization(output: Double) {
    io.setDriveOpenLoop(output)
    io.setTurnPosition(Rotation2d())
  }

  /** Disables all outputs to motors.  */
  fun stop() {
    io.setDriveOpenLoop(0.0)
    io.setTurnOpenLoop(0.0)
  }

  val angle: Rotation2d
    /** Returns the current turn angle of the module.  */
    get() = inputs.turnPosition

  val positionMeters: Double
    /** Returns the current drive position of the module in meters.  */
    get() = inputs.drivePositionRad * constants.WheelRadius

  val velocityMetersPerSec: Double
    /** Returns the current drive velocity of the module in meters per second.  */
    get() = inputs.driveVelocityRadPerSec * constants.WheelRadius

  val position: SwerveModulePosition
    /** Returns the module position (turn angle and drive position).  */
    get() = SwerveModulePosition(positionMeters, angle)

  val state: SwerveModuleState
    /** Returns the module state (turn angle and drive velocity).  */
    get() = SwerveModuleState(velocityMetersPerSec, angle)

  val odometryTimestamps: DoubleArray
    /** Returns the timestamps of the samples received this cycle.  */
    get() = inputs.odometryTimestamps

  val wheelRadiusCharacterizationPosition: Double
    /** Returns the module position in radians.  */
    get() = inputs.drivePositionRad

  val ffCharacterizationVelocity: Double
    /** Returns the module velocity in rotations/sec (Phoenix native units).  */
    get() = Units.radiansToRotations(inputs.driveVelocityRadPerSec)
}
