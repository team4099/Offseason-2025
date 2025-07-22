package com.team4099.robot2025.subsystems.superstructure

import edu.wpi.first.math.kinematics.ChassisSpeeds
import org.team4099.lib.units.AngularVelocity
import org.team4099.lib.units.LinearVelocity
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.ElectricalPotential

sealed interface Request {
  sealed interface DrivetrainRequest : Request {
    class OpenLoop(
      val angularVelocity: AngularVelocity,
      val driveVector: Pair<LinearVelocity, LinearVelocity>,
      val fieldOriented: Boolean = true
    ) : DrivetrainRequest

    class ClosedLoop(
      var chassisSpeeds: ChassisSpeeds,
      val chassisAccels: ChassisSpeeds =
        edu.wpi.first.math.kinematics.ChassisSpeeds(0.0, 0.0, 0.0)
    ) : DrivetrainRequest

    class ZeroSensors(val isInAutonomous: Boolean = false) : DrivetrainRequest
    class Idle : DrivetrainRequest

    class LockWheels : DrivetrainRequest
    class Characterize(val voltage: ElectricalPotential) : DrivetrainRequest
  }

  sealed interface VisionRequest : Request {
    class TargetReef() : VisionRequest
    class TargetTag(val tags: Array<Int>) : VisionRequest
  }

  sealed interface ClimberRequest : Request {
    class OpenLoop(
      val climberVoltage: ElectricalPotential,
      val rollersVoltage: ElectricalPotential
    ) : ClimberRequest
    class ClosedLoop(val position: Angle) : ClimberRequest
    class Home() : ClimberRequest
  }
}
