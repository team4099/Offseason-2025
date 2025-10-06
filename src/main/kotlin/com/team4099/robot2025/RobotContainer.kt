package com.team4099.robot2025

import com.ctre.phoenix6.signals.NeutralModeValue
import com.team4099.robot2025.auto.AutonomousSelector
import com.team4099.robot2025.commands.drivetrain.CoolerTargetTagCommand
import com.team4099.robot2025.commands.drivetrain.ResetGyroYawCommand
import com.team4099.robot2025.commands.drivetrain.TeleopDriveCommand
import com.team4099.robot2025.config.ControlBoard
import com.team4099.robot2025.config.constants.Constants
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
import com.team4099.robot2025.subsystems.drivetrain.CommandSwerveDrive
import com.team4099.robot2025.subsystems.drivetrain.TunerConstants
import com.team4099.robot2025.subsystems.elevator.Elevator
import com.team4099.robot2025.subsystems.elevator.ElevatorIOSim
import com.team4099.robot2025.subsystems.elevator.ElevatorIOTalon
import com.team4099.robot2025.subsystems.indexer.Indexer
import com.team4099.robot2025.subsystems.indexer.IndexerIOSim
import com.team4099.robot2025.subsystems.indexer.IndexerIOTalon
import com.team4099.robot2025.subsystems.intake.Intake
import com.team4099.robot2025.subsystems.intake.IntakeIOSim
import com.team4099.robot2025.subsystems.intake.IntakeIOTalonFX
import com.team4099.robot2025.subsystems.superstructure.Superstructure
import com.team4099.robot2025.subsystems.vision.Vision
import com.team4099.robot2025.subsystems.vision.camera.CameraIO
import com.team4099.robot2025.subsystems.vision.camera.CameraIOPhotonvision
import com.team4099.robot2025.util.driver.Jessika
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.ConditionalCommand
import org.team4099.lib.smoothDeadband
import org.team4099.lib.units.base.inches
import org.team4099.lib.units.derived.Angle
import com.team4099.robot2025.subsystems.Arm.Rollers.Rollers as ArmRollers
import com.team4099.robot2025.subsystems.Arm.Rollers.RollersIOSim as ArmRollersIOSim

object RobotContainer {
  private val drivetrain: CommandSwerveDrive = TunerConstants.createDrivetrain()
  private val vision: Vision
  private val elevator: Elevator
  private val arm: Arm
  private val armRollers: ArmRollers
  private val climber: Climber
  private val intake: Intake
  private val indexer: Indexer
  private val canrange: CANRange
  val superstructure: Superstructure

  val driverRumbleState
    get() = canrange.rumbleTrigger || vision.autoAlignReadyRumble

  val operatorRumbleState
    get() = canrange.rumbleTrigger

  init {
    if (RobotBase.isReal()) {
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
            { drivetrain.state.Pose.rotation }
          ),
          CameraIOPhotonvision(
            VisionConstants.CAMERA_NAMES[1],
            VisionConstants.CAMERA_TRANSFORMS[1],
            drivetrain::addVisionMeasurement,
            { drivetrain.state.Pose.rotation }
          ),
        )
    } else {
      elevator = Elevator(ElevatorIOSim)
      arm = Arm(ArmIOSIm)
      armRollers = ArmRollers(ArmRollersIOSim)
      climber = Climber(ClimberIOSim)
      intake = Intake(IntakeIOSim)
      indexer = Indexer(IndexerIOSim)
      canrange = CANRange(object : CANRangeIO {})

      vision = Vision(object : CameraIO {})
    }

    superstructure =
      Superstructure(
        drivetrain, vision, elevator, arm, armRollers, climber, intake, indexer, canrange
      )
  }

  fun mapDefaultCommands() {
    drivetrain.defaultCommand =
      TeleopDriveCommand(
        driver = Jessika(),
        { ControlBoard.forward.smoothDeadband(Constants.Joysticks.THROTTLE_DEADBAND) },
        { ControlBoard.strafe.smoothDeadband(Constants.Joysticks.THROTTLE_DEADBAND) },
        { ControlBoard.turn.smoothDeadband(Constants.Joysticks.TURN_DEADBAND) },
        { ControlBoard.slowMode },
        drivetrain,
      )
  }

  fun zeroSensors(isInAutonomous: Boolean = false) {
    drivetrain.resetRotation(Rotation2d())
  }

  fun setDriveBrakeMode(neutralModeValue: NeutralModeValue = NeutralModeValue.Brake) {
    drivetrain.configNeutralMode(neutralModeValue)
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
      CoolerTargetTagCommand(drivetrain, vision, yTargetOffset = (-12.94 / 2).inches)
    )

    ControlBoard.alignRight.whileTrue(
      ConditionalCommand(
        CoolerTargetTagCommand(drivetrain, vision),
        CoolerTargetTagCommand(drivetrain, vision, yTargetOffset = (12.94 / 2).inches)
      ) {
        superstructure.theoreticalGamePieceArm == Constants.Universal.GamePiece.ALGAE
      }
    )

    ControlBoard.resetGyro.whileTrue(ResetGyroYawCommand(drivetrain))
    ControlBoard.forceIdle.whileTrue(superstructure.requestIdleCommand())
    ControlBoard.eject.whileTrue(superstructure.ejectCommand())
    ControlBoard.resetGamePieceNone.whileTrue(
      superstructure.resetGamepieceCommand(Constants.Universal.GamePiece.NONE)
    )
    ControlBoard.resetGamePieceCoral.whileTrue(
      superstructure.resetGamepieceCommand(Constants.Universal.GamePiece.CORAL)
    )
    ControlBoard.resetGamePieceAlgae.whileTrue(
      superstructure.resetGamepieceCommand(Constants.Universal.GamePiece.ALGAE)
    )

    ControlBoard.test.onTrue(superstructure.overrideFlag(true))
    ControlBoard.test.onFalse(superstructure.overrideFlag(false))
  }

  fun mapTestControls() {}

  fun getAutonomousCommand() =
    AutonomousSelector.getCommand(drivetrain, elevator, superstructure, vision)

  fun getAutonomousLoadingCommand() = AutonomousSelector.getLoadingCommand(drivetrain)

  fun resetGyroYawCommand(angle: Angle): Command = ResetGyroYawCommand(drivetrain, angle)

  fun mapTunableCommands() {}
}
