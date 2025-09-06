package com.team4099.robot2025

import com.team4099.robot2023.subsystems.vision.camera.CameraIO
import com.team4099.robot2023.subsystems.vision.camera.CameraIOPhotonvision
import com.team4099.robot2025.auto.AutonomousSelector
import com.team4099.robot2025.commands.drivetrain.DrivePathOTF
import com.team4099.robot2025.commands.drivetrain.ResetGyroYawCommand
import com.team4099.robot2025.commands.drivetrain.TeleopDriveCommand
import com.team4099.robot2025.config.ControlBoard
import com.team4099.robot2025.config.constants.Constants
import com.team4099.robot2025.config.constants.VisionConstants
import com.team4099.robot2025.subsystems.Arm.Arm
import com.team4099.robot2025.subsystems.Arm.ArmIOSIm
import com.team4099.robot2025.subsystems.Arm.ArmIOTalon
import com.team4099.robot2025.subsystems.canRange.CANRange
import com.team4099.robot2025.subsystems.canRange.CANRangeIO
import com.team4099.robot2025.subsystems.canRange.CANRangeReal
import com.team4099.robot2025.subsystems.climber.Climber
import com.team4099.robot2025.subsystems.climber.ClimberIOSim
import com.team4099.robot2025.subsystems.climber.ClimberIOTalon
import com.team4099.robot2025.subsystems.drivetrain.drive.Drivetrain
import com.team4099.robot2025.subsystems.drivetrain.drive.DrivetrainIOReal
import com.team4099.robot2025.subsystems.drivetrain.drive.DrivetrainIOSim
import com.team4099.robot2025.subsystems.drivetrain.gyro.GyroIO
import com.team4099.robot2025.subsystems.drivetrain.gyro.GyroIOPigeon2
import com.team4099.robot2025.subsystems.elevator.Elevator
import com.team4099.robot2025.subsystems.elevator.ElevatorIOSim
import com.team4099.robot2025.subsystems.elevator.ElevatorIOTalon
import com.team4099.robot2025.subsystems.indexer.Indexer
import com.team4099.robot2025.subsystems.indexer.IndexerIOSim
import com.team4099.robot2025.subsystems.indexer.IndexerIOTalon
import com.team4099.robot2025.subsystems.intake.Intake
import com.team4099.robot2025.subsystems.intake.IntakeIOSim
import com.team4099.robot2025.subsystems.intake.IntakeIOTalonFX
import com.team4099.robot2025.subsystems.limelight.LimelightVision
import com.team4099.robot2025.subsystems.limelight.LimelightVisionIO
import com.team4099.robot2025.subsystems.superstructure.Request
import com.team4099.robot2025.subsystems.superstructure.Superstructure
import com.team4099.robot2025.subsystems.vision.Vision
import com.team4099.robot2025.util.driver.Jessika
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.ConditionalCommand
import edu.wpi.first.wpilibj2.command.InstantCommand
import org.team4099.lib.geometry.Pose2d
import org.team4099.lib.pplib.PathPlannerHolonomicDriveController
import org.team4099.lib.smoothDeadband
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.perSecond
import java.util.function.Supplier
import com.team4099.robot2025.subsystems.Arm.Rollers.Rollers as ArmRollers
import com.team4099.robot2025.subsystems.Arm.Rollers.RollersIOSim as ArmRollersIOSim
import com.team4099.robot2025.subsystems.Arm.Rollers.RollersIOTalon as ArmRollersIOTalon
import com.team4099.robot2025.subsystems.superstructure.Request.DrivetrainRequest as DrivetrainRequest

object RobotContainer {
  private val drivetrain: Drivetrain
  private val limelight: LimelightVision
  private val vision: Vision
  private val elevator: Elevator
  private val arm: Arm
  private val armRollers: ArmRollers
  private val climber: Climber
  private val intake: Intake
  private val indexer: Indexer
  private val canrange: CANRange
  val superstructure: Superstructure

  val rumbleState
    get() = canrange.rumbleTrigger

  init {
    if (RobotBase.isReal()) {
      drivetrain = Drivetrain(GyroIOPigeon2, DrivetrainIOReal)
      limelight = LimelightVision(object : LimelightVisionIO {})
      elevator = Elevator(ElevatorIOTalon)
      arm = Arm(ArmIOTalon)
      armRollers = ArmRollers(ArmRollersIOTalon)
      climber = Climber(ClimberIOTalon)
      intake = Intake(IntakeIOTalonFX)
      indexer = Indexer(IndexerIOTalon)
      canrange = CANRange(CANRangeReal)

      vision =
        Vision(
          CameraIOPhotonvision(
            VisionConstants.CAMERA_NAMES[0], VisionConstants.CAMERA_TRANSFORMS[0]
          ),
          CameraIOPhotonvision(
            VisionConstants.CAMERA_NAMES[1], VisionConstants.CAMERA_TRANSFORMS[1]
          )
        )
    } else {
      drivetrain = Drivetrain(object : GyroIO {}, DrivetrainIOSim)
      limelight = LimelightVision(object : LimelightVisionIO {})
      elevator = Elevator(ElevatorIOSim)
      arm = Arm(ArmIOSIm)
      armRollers = ArmRollers(ArmRollersIOSim)
      climber = Climber(ClimberIOSim)
      intake = Intake(IntakeIOSim)
      indexer = Indexer(IndexerIOSim)
      canrange = CANRange(object : CANRangeIO {})

      vision = Vision(object : CameraIO {})
    }

    vision.setDataInterfaces(
      { drivetrain.fieldTRobot },
      { drivetrain.addVisionData(it) },
      { drivetrain.addSpeakerVisionData(it) }
    )
    vision.drivetrainOdometry = { drivetrain.odomTRobot }

    superstructure =
      Superstructure(
        drivetrain,
        vision,
        limelight,
        elevator,
        arm,
        armRollers,
        climber,
        intake,
        indexer,
        canrange
      )

    limelight.poseSupplier = { drivetrain.odomTRobot }
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

  fun zeroSteering() {
    drivetrain.zeroSteering()
  }

  fun zeroSensors(isInAutonomous: Boolean = false) {
    drivetrain.currentRequest = DrivetrainRequest.ZeroSensors(isInAutonomous)
  }

  fun zeroAngle(toAngle: Angle) {
    drivetrain.zeroGyroYaw(toAngle)
  }

  fun setSteeringCoastMode() {
    drivetrain.swerveModules.forEach { it.setSteeringBrakeMode(false) }
  }
  fun setSteeringBrakeMode() {
    drivetrain.swerveModules.forEach { it.setSteeringBrakeMode(true) }
  }

  fun setDriveCoastMode() {
    drivetrain.swerveModules.forEach { it.setDriveBrakeMode(false) }
  }

  fun setDriveBrakeMode() {
    drivetrain.swerveModules.forEach { it.setDriveBrakeMode(true) }
  }

  fun requestIdle() {
    superstructure.currentRequest = Request.SuperstructureRequest.Idle()
  }

  fun mapTeleopControls() {
    ControlBoard.intakeCoral.whileTrue(superstructure.intakeCoralCommand())
    ControlBoard.score.whileTrue(superstructure.scoreCommand())
    ControlBoard.climbExtend.whileTrue(superstructure.climbExtendCommand())
    ControlBoard.climbRetract.whileTrue(superstructure.climbRetractCommand())

    ControlBoard.prepL1OrAlgaeGround.whileTrue(superstructure.prepL1OrAlgaeGroundCommand())
    ControlBoard.prepL2OrProcessor.whileTrue(superstructure.prepL2OrProcessorCommand())
    ControlBoard.prepL3OrAlgaeReef.whileTrue(superstructure.prepL3OrAlgaeReefCommand())
    ControlBoard.prepL4OrBarge.whileTrue(superstructure.prepL4OrBargeCommand())

    // todo align commands need to change to utilize superstructure.theoreticalGamePieceArm
    ControlBoard.alignLeft.whileTrue(object : Command() {}) // todo add auto align left
    ControlBoard.alignRight.whileTrue(object : Command() {}) // todo add auto align right
    ControlBoard.alignCenter.whileTrue(object : Command() {}) // todo add auto align center

    ControlBoard.resetGyro.whileTrue(ResetGyroYawCommand(drivetrain))
    ControlBoard.forceIdle.whileTrue(superstructure.requestIdleCommand())
    ControlBoard.eject.whileTrue(superstructure.ejectCommand())

    ControlBoard.test.onTrue(
      ConditionalCommand(
        DrivePathOTF(
          drivetrain,
          { ControlBoard.forward.smoothDeadband(Constants.Joysticks.THROTTLE_DEADBAND) },
          { ControlBoard.strafe.smoothDeadband(Constants.Joysticks.THROTTLE_DEADBAND) },
          { ControlBoard.turn.smoothDeadband(Constants.Joysticks.TURN_DEADBAND) },
          { drivetrain.odomTRobot.pose2d },
          listOf(
            Supplier { Pose2d(4.748.meters, 1.56.meters, 27.216.degrees) },
            Supplier { Pose2d(6.1.meters, 2.996.meters, 87.degrees) },
            Supplier { Pose2d(6.1.meters, 4.093.meters, 90.degrees) }
          ),
          PathPlannerHolonomicDriveController.Companion.GoalEndState(
            0.0.meters.perSecond, 3.14.radians
          )
        ),
        InstantCommand()
      ) { RobotBase.isSimulation() }
    )
  }

  fun mapTestControls() {}

  fun getAutonomousCommand() =
    AutonomousSelector.getCommand(drivetrain, elevator, superstructure, vision)

  fun getAutonomousLoadingCommand() = AutonomousSelector.getLoadingCommand(drivetrain)

  fun resetGyroYawCommand(angle: Angle): Command = ResetGyroYawCommand(drivetrain, angle)

  fun mapTunableCommands() {}
}
