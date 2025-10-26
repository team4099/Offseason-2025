package com.team4099.robot2025.subsystems.drivetrain

import com.ctre.phoenix6.configs.CANcoderConfiguration
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.swerve.SwerveModuleConstants
import com.team4099.lib.phoenix6.PhoenixUtil
import com.team4099.robot2025.subsystems.drivetrain.generated.TunerConstants
import edu.wpi.first.units.Units.Radians
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation
import org.team4099.lib.units.derived.radians

class ModuleIOTalonFXSim(
  constants: SwerveModuleConstants<TalonFXConfiguration?, TalonFXConfiguration?, CANcoderConfiguration?>,
  val simulation: SwerveModuleSimulation
) : ModuleIOTalonFX(constants) {
  init {
    simulation.useDriveMotorController(PhoenixUtil.TalonFXMotorControllerSim(driveTalon))
    simulation.useSteerMotorController(PhoenixUtil.TalonFXMotorControllerWithRemoteCancoderSim(turnTalon, cancoder))
  }

  override fun updateInputs(inputs: ModuleIO.ModuleIOInputs) {
    super.updateInputs(inputs)

    inputs.odometryTimestamps = PhoenixUtil.simulationOdometryTimeStamps
    inputs.odometryDrivePositions = simulation.cachedDriveWheelFinalPositions.map { it.`in`(Radians).radians }.toTypedArray()
    inputs.odometryTurnPositions = simulation.cachedSteerAbsolutePositions.map { it.radians.radians }.toTypedArray()
  }

  companion object {
    fun generateModules(simulation: SwerveDriveSimulation): Array<ModuleIO> {
      return arrayOf(
        ModuleIOTalonFXSim(TunerConstants.FrontLeft, simulation.modules[0]),
        ModuleIOTalonFXSim(TunerConstants.FrontRight, simulation.modules[1]),
        ModuleIOTalonFXSim(TunerConstants.BackLeft, simulation.modules[2]),
        ModuleIOTalonFXSim(TunerConstants.BackRight, simulation.modules[3])
      )
    }
  }
}