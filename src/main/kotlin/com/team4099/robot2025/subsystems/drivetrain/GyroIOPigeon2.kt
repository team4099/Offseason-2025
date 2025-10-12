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

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.StatusCode
import com.ctre.phoenix6.StatusSignal
import com.ctre.phoenix6.configs.Pigeon2Configuration
import com.ctre.phoenix6.hardware.Pigeon2
import com.team4099.robot2025.subsystems.drivetrain.generated.TunerConstants
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.util.Units
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import java.util.Queue

/** IO implementation for Pigeon 2.  */
class GyroIOPigeon2 : GyroIO {
  private val pigeon: Pigeon2 = Pigeon2(
    TunerConstants.CTREDrivetrainConstants.Pigeon2Id,
    TunerConstants.CTREDrivetrainConstants.CANBusName
  )
  private val yaw: StatusSignal<Angle> = pigeon.yaw
  private val yawPositionQueue: Queue<Double>
  private val yawTimestampQueue: Queue<Double>
  private val yawVelocity: StatusSignal<AngularVelocity> = pigeon.angularVelocityZWorld

  init {
    pigeon.configurator.apply(Pigeon2Configuration())
    pigeon.configurator.setYaw(0.0)
    yaw.setUpdateFrequency(Drive.ODOMETRY_FREQUENCY)
    yawVelocity.setUpdateFrequency(50.0)
    pigeon.optimizeBusUtilization()
    yawTimestampQueue = PhoenixOdometryThread.instance.makeTimestampQueue()
    yawPositionQueue = PhoenixOdometryThread.instance.registerSignal(pigeon.yaw)
  }

  override fun updateInputs(inputs: GyroIO.GyroIOInputs) {
    inputs.connected = BaseStatusSignal.refreshAll(yaw, yawVelocity) == StatusCode.OK
    inputs.yawPosition = Rotation2d.fromDegrees(yaw.valueAsDouble)
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(yawVelocity.valueAsDouble)

    inputs.odometryYawTimestamps =
      yawTimestampQueue.stream().mapToDouble { value: Double -> value }.toArray()
    inputs.odometryYawPositions =
      yawPositionQueue.stream()
        .map { value: Double -> Rotation2d.fromDegrees(value) }
        .toArray() as Array<Rotation2d>
    yawTimestampQueue.clear()
    yawPositionQueue.clear()
  }
}
