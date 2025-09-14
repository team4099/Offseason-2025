package com.team4099.robot2025.commands.drivetrain

import com.ctre.phoenix6.swerve.SwerveModule
import com.team4099.robot2025.subsystems.drivetrain.CommandSwerveDrive
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.wpilibj2.command.Command
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.inRotation2ds

class SwerveModuleTuningCommand(
  val drivetrain: CommandSwerveDrive,
  val steeringPosition: () -> Angle
) : Command() {
  init {
    addRequirements(drivetrain)
  }

  override fun execute() {
    for (module in drivetrain.modules) {
      module.apply(
        SwerveModule.ModuleRequest()
          .withState(SwerveModuleState(0.0, steeringPosition().inRotation2ds))
      )
    }
  }

  override fun isFinished(): Boolean {
    return false
  }
}
