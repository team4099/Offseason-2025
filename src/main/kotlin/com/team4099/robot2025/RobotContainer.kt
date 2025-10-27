package com.team4099.robot2025

import com.ctre.phoenix6.signals.NeutralModeValue
import com.team4099.robot2025.auto.AutonomousSelector
import com.team4099.robot2025.commands.drivetrain.CoolerTargetTagCommand
import com.team4099.robot2025.commands.drivetrain.DrivePathOTF
import com.team4099.robot2025.commands.drivetrain.ResetGyroCommand
import com.team4099.robot2025.commands.drivetrain.TeleopDriveCommand
import com.team4099.robot2025.config.ControlBoard
import com.team4099.robot2025.config.constants.Constants
import com.team4099.robot2025.config.constants.Constants.Universal.GamePiece
import com.team4099.robot2025.config.constants.VisionConstants
import com.team4099.robot2025.subsystems.Arm.Arm
import com.team4099.robot2025.subsystems.Arm.ArmIOSIm
import com.team4099.robot2025.subsystems.Arm.ArmIOTalon
import com.team4099.robot2025.subsystems.Arm.Rollers.RollersIOTalon
import com.team4099.robot2025.subsystems.canRange.CANRange
import com.team4099.robot2025.subsystems.canRange.CANRangeIO
import com.team4099.robot2025.subsystems.canRange.CANRangeReal
import com.team4099.robot2025.subsystems.climber.Climber
import com.team4099.robot2025.subsystems.climber.ClimberIO
import com.team4099.robot2025.subsystems.climber.ClimberIOSim
import com.team4099.robot2025.subsystems.drivetrain.Drive
import com.team4099.robot2025.subsystems.drivetrain.GyroIOPigeon2
import com.team4099.robot2025.subsystems.drivetrain.GyroIOSim
import com.team4099.robot2025.subsystems.drivetrain.ModuleIOTalonFXReal
import com.team4099.robot2025.subsystems.drivetrain.ModuleIOTalonFXSim
import com.team4099.robot2025.subsystems.elevator.Elevator
import com.team4099.robot2025.subsystems.elevator.ElevatorIOSim
import com.team4099.robot2025.subsystems.elevator.ElevatorIOTalon
import com.team4099.robot2025.subsystems.indexer.Indexer
import com.team4099.robot2025.subsystems.indexer.IndexerIOSim
import com.team4099.robot2025.subsystems.indexer.IndexerIOTalon
import com.team4099.robot2025.subsystems.intake.Intake
import com.team4099.robot2025.subsystems.intake.IntakeIOSim
import com.team4099.robot2025.subsystems.intake.IntakeIOTalonFX
import com.team4099.robot2025.subsystems.led.Led
import com.team4099.robot2025.subsystems.led.LedIO
import com.team4099.robot2025.subsystems.led.LedIOCandle
import com.team4099.robot2025.subsystems.superstructure.Superstructure
import com.team4099.robot2025.subsystems.vision.Vision
import com.team4099.robot2025.subsystems.vision.camera.CameraIOPVSim
import com.team4099.robot2025.subsystems.vision.camera.CameraIOPhotonvision
import com.team4099.robot2025.util.driver.Jessika
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.ConditionalCommand
import org.ironmaple.simulation.SimulatedArena
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation
import org.littletonrobotics.junction.Logger
import org.team4099.lib.geometry.Pose2d
import org.team4099.lib.pplib.PathPlannerHolonomicDriveController
import org.team4099.lib.smoothDeadband
import org.team4099.lib.units.base.inches
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.perSecond
import java.util.function.Supplier
import com.team4099.robot2025.subsystems.Arm.Rollers.Rollers as ArmRollers
import com.team4099.robot2025.subsystems.Arm.Rollers.RollersIOSim as ArmRollersIOSim

object RobotContainer {
  private val drivetrain: Drive
  private val vision: Vision
  private val elevator: Elevator
  private val arm: Arm
  private val armRollers: ArmRollers
  private val climber: Climber
  private val intake: Intake
  private val indexer: Indexer
  private val canrange: CANRange
  private val led: Led
  val superstructure: Superstructure

  val driverRumbleState
    get() = canrange.rumbleTrigger || vision.autoAlignReadyRumble

  val operatorRumbleState
    get() = canrange.rumbleTrigger || vision.autoAlignReadyRumble

  var driveSimulation: SwerveDriveSimulation? = null

  init {
    if (RobotBase.isReal()) {
      drivetrain = Drive(GyroIOPigeon2, ModuleIOTalonFXReal.generateModules(), { pose -> {} })
      elevator = Elevator(ElevatorIOTalon)
      arm = Arm(ArmIOTalon)
      armRollers = ArmRollers(RollersIOTalon)
      climber = Climber(object : ClimberIO {})
      intake = Intake(IntakeIOTalonFX)
      indexer = Indexer(IndexerIOTalon)
      canrange = CANRange(CANRangeReal)

      vision =
        Vision(
          CameraIOPhotonvision(
            VisionConstants.CAMERA_NAMES[0],
            VisionConstants.CAMERA_TRANSFORMS[0],
            drivetrain::addVisionMeasurement,
            { drivetrain.pose.rotation },
          ),
          CameraIOPhotonvision(
            VisionConstants.CAMERA_NAMES[1],
            VisionConstants.CAMERA_TRANSFORMS[1],
            drivetrain::addVisionMeasurement,
            { drivetrain.pose.rotation },
            //            { vision.isAutoAligning }
          ),
        )

      led =
        Led(
          { GamePiece.NONE },
          { vision.isAligned },
          { vision.isAutoAligning },
          { Superstructure.Companion.SuperstructureStates.UNINITIALIZED },
          LedIOCandle(Constants.Candle.CANDLE_ID_1),
          LedIOCandle(Constants.Candle.CANDLE_ID_2)
        )
    } else {
      driveSimulation =
        SwerveDriveSimulation(Drive.mapleSimConfig, Pose2d(3.meters, 3.meters, 0.radians).pose2d)
      SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation)

      drivetrain =
        Drive(
          GyroIOSim(driveSimulation!!.gyroSimulation),
          ModuleIOTalonFXSim.generateModules(driveSimulation!!),
          driveSimulation!!::setSimulationWorldPose
        )
      elevator = Elevator(ElevatorIOSim)
      arm = Arm(ArmIOSIm)
      armRollers = ArmRollers(ArmRollersIOSim)
      climber = Climber(ClimberIOSim)
      intake = Intake(IntakeIOSim)
      indexer = Indexer(IndexerIOSim)
      canrange = CANRange(object : CANRangeIO {})

      vision =
        Vision(
          CameraIOPVSim(
            VisionConstants.CAMERA_NAMES[0],
            VisionConstants.CAMERA_TRANSFORMS[0],
            drivetrain::addVisionMeasurement,
            { drivetrain.rotation }
          ),
          CameraIOPVSim(
            VisionConstants.CAMERA_NAMES[1],
            VisionConstants.CAMERA_TRANSFORMS[1],
            drivetrain::addVisionMeasurement,
            { drivetrain.rotation }
          ),
          poseSupplier = { drivetrain.pose.pose2d }
        )

      led =
        Led(
          { GamePiece.NONE },
          { vision.isAligned },
          { vision.isAutoAligning },
          { Superstructure.Companion.SuperstructureStates.UNINITIALIZED },
          object : LedIO {}
        )
    }

    superstructure =
      Superstructure(
        drivetrain, vision, elevator, arm, armRollers, climber, intake, indexer, canrange, led
      )

    //    led.isAlignedSupplier = Supplier { vision.isAligned }
    led.gamePieceArmSupplier = Supplier { superstructure.theoreticalGamePieceArm }
    led.stateSupplier = Supplier { superstructure.currentState }
  }

  fun mapDefaultCommands() {
    drivetrain.defaultCommand =
      TeleopDriveCommand(
        driver = Jessika(),
        { ControlBoard.forward.smoothDeadband(Constants.Joysticks.THROTTLE_DEADBAND) },
        { ControlBoard.strafe.smoothDeadband(Constants.Joysticks.THROTTLE_DEADBAND) },
        { ControlBoard.turn.smoothDeadband(Constants.Joysticks.TURN_DEADBAND) },
        {
          ControlBoard.slowMode ||
            superstructure.currentState ==
            Superstructure.Companion.SuperstructureStates.GROUND_INTAKE_CORAL ||
            superstructure.currentState ==
            Superstructure.Companion.SuperstructureStates.INTAKE_ALGAE ||
            superstructure.currentState ==
            Superstructure.Companion.SuperstructureStates.SCORE_ALGAE &&
            superstructure.algaeScoringLevel ==
            Constants.Universal.AlgaeScoringLevel.BARGE
        },
        drivetrain,
      )
  }

  fun zeroSensors(isInAutonomous: Boolean = false) {
    drivetrain.pose = Pose2d(drivetrain.pose.x, drivetrain.pose.y, 0.radians)
  }

  fun setDriveBrakeMode(neutralModeValue: NeutralModeValue = NeutralModeValue.Brake) {
    //    drivetrain.configNeutralMode(neutralModeValue)
  }

  fun mapTeleopControls() {
    ControlBoard.intakeCoral.whileTrue(superstructure.intakeCoralCommand())
    ControlBoard.score.whileTrue(superstructure.scoreCommand())
    //    ControlBoard.climbExtend.whileTrue(superstructure.climbExtendCommand())
    //    ControlBoard.climbRetract.whileTrue(superstructure.climbRetractCommand())

    ControlBoard.prepL1OrAlgaeGround.whileTrue(superstructure.prepL1OrAlgaeGroundCommand())
    ControlBoard.prepL2OrProcessor.whileTrue(superstructure.prepL2OrProcessorCommand())
    ControlBoard.prepL3OrAlgaeReef.whileTrue(superstructure.prepL3OrAlgaeReefCommand())
    ControlBoard.prepL4OrBarge.whileTrue(superstructure.prepL4OrBargeCommand())

    ControlBoard.alignLeft.whileTrue(
      ConditionalCommand(
        CoolerTargetTagCommand(drivetrain, vision),
        CoolerTargetTagCommand(drivetrain, vision, yTargetOffset = (12.94 / 2).inches)
      ) {
        superstructure.theoreticalGamePieceArm == GamePiece.ALGAE
      }
    )

    ControlBoard.alignRight.whileTrue(
      ConditionalCommand(
        CoolerTargetTagCommand(drivetrain, vision),
        CoolerTargetTagCommand(drivetrain, vision, yTargetOffset = (-12.94 / 2).inches)
      ) {
        superstructure.theoreticalGamePieceArm == GamePiece.ALGAE
      }
    )

    ControlBoard.resetGyro.whileTrue(ResetGyroCommand(drivetrain))
    ControlBoard.forceIdle.whileTrue(superstructure.requestIdleCommand())
    ControlBoard.eject.whileTrue(superstructure.ejectCommand())

    ControlBoard.test.onTrue(
      DrivePathOTF(
        drivetrain,
        { ControlBoard.forward.smoothDeadband(Constants.Joysticks.THROTTLE_DEADBAND) },
        { ControlBoard.strafe.smoothDeadband(Constants.Joysticks.THROTTLE_DEADBAND) },
        { ControlBoard.turn.smoothDeadband(Constants.Joysticks.TURN_DEADBAND) },
        { drivetrain.pose.pose2d },
        VisionConstants.OTF_PATHS[vision.lastTrigVisionUpdate.targetTagID]
          ?: listOf(
            Supplier { Pose2d(4.748.meters, 1.56.meters, 27.216.degrees) },
            Supplier { Pose2d(6.1.meters, 2.996.meters, 87.degrees) },
            Supplier { Pose2d(6.1.meters, 4.093.meters, 90.degrees) }
          ),
        0.0.degrees,
        PathPlannerHolonomicDriveController.Companion.GoalEndState(
          0.0.meters.perSecond, 180.degrees
        )
      )
    )

    ControlBoard.resetGamePieceNone.whileTrue(superstructure.resetGamepieceCommand(GamePiece.NONE))
    ControlBoard.resetGamePieceCoral.whileTrue(
      superstructure.resetGamepieceCommand(GamePiece.CORAL)
    )
    ControlBoard.resetGamePieceAlgae.whileTrue(
      superstructure.resetGamepieceCommand(GamePiece.ALGAE)
    )

    ControlBoard.test.whileTrue(ResetGyroCommand(drivetrain))
  }

  fun mapTestControls() {}

  fun getAutonomousCommand() =
    AutonomousSelector.getCommand(drivetrain, elevator, superstructure, vision)

  fun getAutonomousLoadingCommand() = AutonomousSelector.getLoadingCommand(drivetrain)

  fun mapTunableCommands() {}

  fun resetSimulationField() {
    if (!RobotBase.isSimulation()) return

    driveSimulation!!.setSimulationWorldPose(Pose2d(3.meters, 3.meters, 0.radians).pose2d)
    SimulatedArena.getInstance().resetFieldForAuto()
  }

  fun updateSimulation() {
    if (!RobotBase.isSimulation()) return

    SimulatedArena.getInstance().simulationPeriodic()
    Logger.recordOutput("FieldSimulation/RobotPosition", driveSimulation!!.simulatedDriveTrainPose)
    SimulatedArena.getInstance().getGamePiecesArrayByType("Coral").mapIndexed { index, value ->
      Logger.recordOutput("FieldSimulation/Coral/$index", value)
    }
    SimulatedArena.getInstance().getGamePiecesArrayByType("Algae").mapIndexed { index, value ->
      Logger.recordOutput("FieldSimulation/Algae/$index", value)
    }
  }
}
