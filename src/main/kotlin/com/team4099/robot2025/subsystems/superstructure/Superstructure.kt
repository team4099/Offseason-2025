package com.team4099.robot2025.subsystems.superstructure

import com.team4099.lib.hal.Clock
import com.team4099.robot2025.config.constants.ArmConstants
import com.team4099.robot2025.config.constants.ClimberConstants
import com.team4099.robot2025.config.constants.Constants
import com.team4099.robot2025.config.constants.ElevatorConstants
import com.team4099.robot2025.config.constants.GyroConstants
import com.team4099.robot2025.config.constants.IntakeConstants
import com.team4099.robot2025.subsystems.Arm.Arm
import com.team4099.robot2025.subsystems.Arm.ArmTunableValues
import com.team4099.robot2025.subsystems.canRange.CANRange
import com.team4099.robot2025.subsystems.climber.Climber
import com.team4099.robot2025.subsystems.drivetrain.CommandSwerveDrive
import com.team4099.robot2025.subsystems.elevator.Elevator
import com.team4099.robot2025.subsystems.elevator.ElevatorTunableValues
import com.team4099.robot2025.subsystems.indexer.Indexer
import com.team4099.robot2025.subsystems.intake.Intake
import com.team4099.robot2025.subsystems.intake.IntakeTunableValues
import com.team4099.robot2025.subsystems.led.Led
import com.team4099.robot2025.subsystems.limelight.LimelightVision
import com.team4099.robot2025.subsystems.superstructure.Request.SuperstructureRequest
import com.team4099.robot2025.subsystems.vision.Vision
import com.team4099.robot2025.util.CustomLogger
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.ConditionalCommand
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.Logger
import org.team4099.lib.geometry.Pose3d
import org.team4099.lib.geometry.Rotation3d
import org.team4099.lib.geometry.Translation3d
import org.team4099.lib.units.base.inInches
import org.team4099.lib.units.base.inMilliseconds
import org.team4099.lib.units.base.inches
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inDegrees
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.derived.volts
import kotlin.math.abs
import kotlin.math.max
import com.team4099.robot2025.config.constants.Constants.Universal.AlgaeIntakeLevel as AlgaeIntakeLevel
import com.team4099.robot2025.config.constants.Constants.Universal.AlgaeScoringLevel as AlgaeScoringLevel
import com.team4099.robot2025.config.constants.Constants.Universal.CoralLevel as CoralLevel
import com.team4099.robot2025.config.constants.Constants.Universal.GamePiece as GamePiece
import com.team4099.robot2025.config.constants.RollersConstants as ArmRollersConstants
import com.team4099.robot2025.subsystems.Arm.Rollers.Rollers as ArmRollers
import com.team4099.robot2025.subsystems.superstructure.Request.RollersRequest as ArmRollersRequest

class Superstructure(
  private val drivetrain: CommandSwerveDrive,
  private val vision: Vision,
  private val limelight: LimelightVision,
  private val elevator: Elevator,
  private val arm: Arm,
  private val armRollers: ArmRollers,
  private val climber: Climber,
  private val intake: Intake,
  private val indexer: Indexer,
  private val canrange: CANRange,
  private val led: Led
) : SubsystemBase() {

  private var overrideFlagForSim = false

  var theoreticalGamePieceArm: GamePiece = GamePiece.CORAL // preload !!
  val theoreticalGamePieceHardstop: GamePiece
    get() =
      if (canrange.inputs.isDetected || (RobotBase.isSimulation() && overrideFlagForSim))
        GamePiece.CORAL
      else GamePiece.NONE

  private var lastCoralScoringLevel: CoralLevel = CoralLevel.NONE
  private var coralScoringLevel: CoralLevel = CoralLevel.NONE

  private var lastAlgaeIntakeLevel: AlgaeIntakeLevel = AlgaeIntakeLevel.NONE
  private var algaeIntakeLevel: AlgaeIntakeLevel = AlgaeIntakeLevel.NONE

  private var lastAlgaeScoringLevel: AlgaeScoringLevel = AlgaeScoringLevel.NONE
  private var algaeScoringLevel: AlgaeScoringLevel = AlgaeScoringLevel.NONE
  private var lastPrepLevel: CoralLevel = CoralLevel.NONE

  var currentRequest: SuperstructureRequest = SuperstructureRequest.Idle()
    set(value) {
      when (value) {
        is SuperstructureRequest.IntakeAlgae -> {
          algaeIntakeLevel = value.level
        }
        is SuperstructureRequest.PrepScoreCoral -> {
          coralScoringLevel = value.level
        }
        is SuperstructureRequest.PrepScoreAlgae -> {
          algaeScoringLevel = value.level
        }
        else -> {}
      }
      field = value
    }

  var currentState: SuperstructureStates = SuperstructureStates.UNINITIALIZED

  var isAtRequestedState: Boolean = false

  private var lastTransitionTime = Clock.fpgaTime

  override fun periodic() {
    val armStartTime = Clock.realTimestamp
    arm.periodic()
    CustomLogger.recordOutput(
      "LoggedRobot/Subsystems/armLoopTimeMS", (Clock.realTimestamp - armStartTime).inMilliseconds
    )

    val armRollersStartTime = Clock.realTimestamp
    armRollers.periodic()
    CustomLogger.recordOutput(
      "LoggedRobot/Subsystems/armRollersLoopTimeMS",
      (Clock.realTimestamp - armRollersStartTime).inMilliseconds
    )

    val canRangeStartTime = Clock.realTimestamp
    canrange.periodic()
    CustomLogger.recordOutput(
      "LoggedRobot/Subsystems/canRangeLoopTimeMS",
      (Clock.realTimestamp - canRangeStartTime).inMilliseconds
    )

    val climberStartTime = Clock.realTimestamp
    climber.periodic()
    CustomLogger.recordOutput(
      "LoggedRobot/Subsystems/climberLoopTimeMS",
      (Clock.realTimestamp - climberStartTime).inMilliseconds
    )

    val elevatorStartTime = Clock.realTimestamp
    elevator.periodic()
    CustomLogger.recordOutput(
      "LoggedRobot/Subsystems/elevatorLoopTimeMS",
      (Clock.realTimestamp - elevatorStartTime).inMilliseconds
    )

    val indexerStartTime = Clock.realTimestamp
    indexer.periodic()
    CustomLogger.recordOutput(
      "LoggedRobot/Subsystems/indexerLoopTimeMS",
      (Clock.realTimestamp - indexerStartTime).inMilliseconds
    )

    val intakeStartTime = Clock.realTimestamp
    intake.periodic()
    CustomLogger.recordOutput(
      "LoggedRobot/Subsystems/intakeLoopTimeMS",
      (Clock.realTimestamp - intakeStartTime).inMilliseconds
    )

    val ledStartTime = Clock.realTimestamp
    led.periodic()
    CustomLogger.recordOutput(
      "LoggedRobot/Subsystems/ledLoopTimeMS",
      (Clock.realTimestamp - ledStartTime).inMilliseconds
    )

    /** 0 - first stage 1 - carriage 2 - intake pivot 3 - arm 4 - climb pivot */
    Logger.recordOutput(
      "SimulatedMechanisms/0",
      Pose3d(
        Translation3d(
          0.0.inches,
          0.0.inches,
          max(
            elevator.inputs.elevatorPosition.inInches -
              ElevatorConstants.FIRST_STAGE_HEIGHT.inInches,
            0.0
          )
            .inches
        ),
        Rotation3d()
      )
        .pose3d
    )

    CustomLogger.recordDebugOutput(
      "SimulatedMechanisms/1",
      Pose3d(
        Translation3d(0.0.inches, 0.0.inches, elevator.inputs.elevatorPosition),
        Rotation3d()
      )
        .pose3d
    )

    CustomLogger.recordDebugOutput(
      "SimulatedMechanisms/2",
      Pose3d(
        Translation3d((-11.75).inches, 0.0.inches, 12.5747.inches),
        Rotation3d(
          0.0.degrees,
          IntakeConstants.ANGLES.INTAKE_ANGLE - intake.inputs.pivotPosition,
          0.0.degrees
        ) // model starts in intaking position
      )
        .pose3d
    )

    CustomLogger.recordDebugOutput(
      "SimulatedMechanisms/3",
      Pose3d(
        Translation3d(
          0.0.inches,
          0.0.inches,
          elevator.inputs.elevatorPosition + ElevatorConstants.CARRIAGE_TO_BOTTOM_SIM
        ),
        Rotation3d(
          0.0.degrees,
          ArmConstants.ANGLES.SIM_MECH_OFFSET - arm.inputs.armPosition,
          0.0.degrees
        )
      )
        .pose3d
    )

    CustomLogger.recordDebugOutput(
      "SimulatedMechanisms/4",
      Pose3d(
        Translation3d(0.008.meters, 0.35.meters, 0.373.meters),
        Rotation3d(
          -(
            -ClimberConstants.SIM_CLIMBED_ANGLE.inDegrees *
              abs(
                climber.inputs.climberPosition.inDegrees -
                  ClimberConstants.FULLY_EXTENDED_ANGLE.inDegrees
              ) /
              ClimberConstants.FULLY_EXTENDED_ANGLE.inDegrees
            )
            .degrees, // ratchet to mechanism math
          0.0.degrees,
          0.0.degrees
        )
      )
        .pose3d
    )

    CustomLogger.recordOutput("Superstructure/currentRequest", currentRequest.javaClass.simpleName)
    CustomLogger.recordOutput("Superstructure/currentState", currentState.name)
    CustomLogger.recordOutput("Superstructure/isAtAllTargetedPositions", isAtRequestedState)
    CustomLogger.recordOutput("Superstructure/theoreticalGamePieceArm", theoreticalGamePieceArm)
    CustomLogger.recordOutput(
      "Superstructure/theoreticalGamePieceHardstop", theoreticalGamePieceHardstop
    )
    CustomLogger.recordOutput("Superstructure/coralScoringLevel", coralScoringLevel)
    CustomLogger.recordOutput("Superstructure/algaeIntakeLevel", algaeIntakeLevel)
    CustomLogger.recordOutput("Superstructure/algaeScoringLevel", algaeScoringLevel)
    CustomLogger.recordOutput("Superstructure/lastPrepLevel", lastPrepLevel)

    if (drivetrain.rotation3d.x.radians > GyroConstants.ANTI_TILT_THRESHOLD_ROLL ||
      drivetrain.rotation3d.y.radians > GyroConstants.ANTI_TILT_THRESHOLD_PITCH
    ) {
      currentRequest = SuperstructureRequest.Idle()
    }

    var nextState = currentState
    when (currentState) {
      // General States
      SuperstructureStates.UNINITIALIZED -> {
        nextState =
          if (Constants.Tuning.TUNING_MODE) SuperstructureStates.TUNING
          else if (RobotBase.isSimulation()) SuperstructureStates.IDLE
          else SuperstructureStates.HOME_PREP
      }
      SuperstructureStates.HOME_PREP -> {
        arm.currentRequest = Request.ArmRequest.ClosedLoop(ArmConstants.ANGLES.HOME_ANGLE)

        if (arm.isAtTargetedPosition) {
          nextState = SuperstructureStates.HOME
        }
      }
      SuperstructureStates.HOME -> {
        elevator.currentRequest = Request.ElevatorRequest.Home()

        if (elevator.isHomed) {
          nextState =
            if (Constants.Tuning.TUNING_MODE) SuperstructureStates.TUNING
            else SuperstructureStates.IDLE
        }
      }
      SuperstructureStates.TUNING -> {
        //  if (currentRequest is SuperstructureRequest.Idle) nextState = SuperstructureStates.IDLE
      }
      SuperstructureStates.IDLE -> {
        climber.currentRequest = Request.ClimberRequest.OpenLoop(0.0.volts, 0.0.volts)
        intake.currentRequest =
          Request.IntakeRequest.TargetingPosition(
            IntakeTunableValues.idlePosition.get(), IntakeTunableValues.idleRollerVoltage.get()
          )
        indexer.currentRequest = Request.IndexerRequest.Idle()

        when (theoreticalGamePieceArm) {
          GamePiece.ALGAE -> {
            // arm should move first in case elevator ends up moving down
            arm.currentRequest =
              Request.ArmRequest.ClosedLoop(ArmTunableValues.Angles.idleAlgaeAngle.get())
            armRollers.currentRequest =
              ArmRollersRequest.OpenLoop(ArmRollersConstants.IDLE_ALGAE_VOLTAGE)
            if (arm.isAtTargetedPosition) {
              elevator.currentRequest =
                Request.ElevatorRequest.ClosedLoop(
                  ElevatorTunableValues.Heights.idleAlgaeHeight.get()
                )
            }
          }
          else -> {
            val armIdleAngle =
              if (theoreticalGamePieceArm == GamePiece.CORAL)
                ArmTunableValues.Angles.idleCoralAngle.get()
              else ArmTunableValues.Angles.idleAngle.get()

            val elevatorIdlePosition =
              if (theoreticalGamePieceArm == GamePiece.CORAL)
                ElevatorTunableValues.Heights.idleCoralHeight.get()
              else ElevatorTunableValues.Heights.idleHeight.get()

            armRollers.currentRequest =
              Request.RollersRequest.OpenLoop(
                if (theoreticalGamePieceArm == GamePiece.CORAL)
                  ArmRollersConstants.IDLE_CORAL_VOLTAGE
                else ArmRollersConstants.IDLE_VOLTAGE
              )

            // note(nathan): ASSERT IDLE AND IDLE_CORAL > CLEARS_ROBOT
            if (elevator.inputs.elevatorPosition >
              ElevatorConstants.HEIGHTS.ARM_IDLE_PRIORITY_THRESHOLD
            ) {
              arm.currentRequest = Request.ArmRequest.ClosedLoop(armIdleAngle)

              if (arm.isAtTargetedPosition) {
                elevator.currentRequest = Request.ElevatorRequest.ClosedLoop(elevatorIdlePosition)
              }
            } else {
              elevator.currentRequest = Request.ElevatorRequest.ClosedLoop(elevatorIdlePosition)

              if (elevator.isAtTargetedPosition) {
                arm.currentRequest = Request.ArmRequest.ClosedLoop(armIdleAngle)
              }
            }
          }
        }

        // idle to request transitions
        if (elevator.isAtTargetedPosition && arm.isAtTargetedPosition)
          nextState =
            if (theoreticalGamePieceArm == GamePiece.NONE &&
              theoreticalGamePieceHardstop == GamePiece.CORAL
            ) {
              currentRequest = SuperstructureRequest.IntakeCoral()
              SuperstructureStates.INTAKE_CORAL_INTO_ARM
            } else
              when (currentRequest) {
                is SuperstructureRequest.Home -> SuperstructureStates.HOME_PREP
                is SuperstructureRequest.IntakeCoral ->
                  SuperstructureStates.GROUND_INTAKE_CORAL
                is SuperstructureRequest.IntakeAlgae -> {
                  if (theoreticalGamePieceArm == GamePiece.NONE)
                    SuperstructureStates.INTAKE_ALGAE
                  else currentState
                }
                is SuperstructureRequest.PrepScoreCoral ->
                  SuperstructureStates.PREP_SCORE_CORAL
                is SuperstructureRequest.PrepScoreAlgae -> {
                  if (theoreticalGamePieceArm == GamePiece.ALGAE)
                    SuperstructureStates.PREP_SCORE_ALGAE
                  else currentState
                }
                is SuperstructureRequest.ExtendClimb -> SuperstructureStates.CLIMB_EXTEND
                is SuperstructureRequest.RetractClimb -> SuperstructureStates.CLIMB_RETRACT
                is SuperstructureRequest.Eject -> SuperstructureStates.EJECT
                else -> currentState
              }
      }
      SuperstructureStates.GROUND_INTAKE_CORAL -> {
        intake.currentRequest =
          Request.IntakeRequest.TargetingPosition(
            IntakeTunableValues.coralPosition.get(),
            IntakeTunableValues.coralRollerVoltage.get()
          )
        indexer.currentRequest = Request.IndexerRequest.Index()

        if (currentRequest is SuperstructureRequest.Eject) {
          nextState = SuperstructureStates.EJECT
        } else if (theoreticalGamePieceHardstop == GamePiece.CORAL ||
          currentRequest is SuperstructureRequest.Idle
        ) {
          currentRequest = SuperstructureRequest.Idle()
          nextState = SuperstructureStates.GROUND_INTAKE_CORAL_CLEANUP
        }
      }
      SuperstructureStates.GROUND_INTAKE_CORAL_CLEANUP -> {
        intake.currentRequest =
          Request.IntakeRequest.TargetingPosition(
            IntakeTunableValues.idlePosition.get(), IntakeConstants.Rollers.IDLE_VOLTAGE
          )
        indexer.currentRequest = Request.IndexerRequest.Idle()

        if (intake.isAtTargetedPosition) {
          nextState =
            if (theoreticalGamePieceArm == GamePiece.NONE &&
              theoreticalGamePieceHardstop ==
              GamePiece.CORAL
            ) { // if not holding a game piece, intake it now
              SuperstructureStates.INTAKE_CORAL_INTO_ARM
            } else { // if holding an algae or we got here interrupted (force idle)
              SuperstructureStates.IDLE
            }
        }
      }
      SuperstructureStates.INTAKE_CORAL_INTO_ARM -> {
        arm.currentRequest =
          Request.ArmRequest.ClosedLoop(ArmTunableValues.Angles.hardstopIntakeAngle.get())

        if (arm.isAtTargetedPosition) {
          elevator.currentRequest =
            Request.ElevatorRequest.ClosedLoop(
              ElevatorTunableValues.Heights.intakeCoralHeight.get()
            )
          armRollers.currentRequest =
            ArmRollersRequest.OpenLoop(ArmRollersConstants.INTAKE_CORAL_VOLTAGE)

          if (RobotBase.isReal() &&
            armRollers.hasCoral &&
            Clock.fpgaTime - lastTransitionTime > ArmRollersConstants.CORAL_DETECTION_THRESHOLD ||
            RobotBase.isSimulation() && !overrideFlagForSim
          ) {
            theoreticalGamePieceArm = GamePiece.CORAL
          }
        }

        if (currentRequest is SuperstructureRequest.Eject) {
          nextState = SuperstructureStates.EJECT
        } else if (currentRequest is SuperstructureRequest.Idle ||
          arm.isAtTargetedPosition && theoreticalGamePieceArm == GamePiece.CORAL
        ) {
          currentRequest = SuperstructureRequest.Idle()
          nextState = SuperstructureStates.IDLE
        }
      }
      SuperstructureStates.INTAKE_ALGAE -> {
        armRollers.currentRequest =
          ArmRollersRequest.OpenLoop(ArmRollersConstants.INTAKE_ALGAE_VOLTAGE)

        // g-intake algae is so weird so check that first
        if (algaeIntakeLevel == AlgaeIntakeLevel.GROUND) {
          // case 1: elevator is too low, move it up first
          // check: elevator is too low, arm hasnt moved out yet
          // solution: raise the elevator enough to get over that
          if (!elevator.clearsRobot &&
            arm.inputs.armPosition <= ArmConstants.ANGLES.ARM_GUARENTEED_OVER_BATTERY
          ) {
            elevator.currentRequest =
              Request.ElevatorRequest.ClosedLoop(
                ElevatorConstants.HEIGHTS.CLEARS_ROBOT + 2.0.inches
              )
          } else {
            // case 2: elevator is high enough for arm to move, but arms needs to move out before
            // elevator can move back down again
            arm.currentRequest =
              Request.ArmRequest.ClosedLoop(ArmTunableValues.Angles.algaeGroundIntakeAngle.get())

            if (arm.isAtTargetedPosition) {
              // once arm is out, we can move the elevator back down
              elevator.currentRequest =
                Request.ElevatorRequest.ClosedLoop(
                  ElevatorTunableValues.Heights.intakeAlgaeGroundHeight.get()
                )
            }
          }
        } else {
          // everything else is pretty standard
          elevator.currentRequest =
            Request.ElevatorRequest.ClosedLoop(
              when (algaeIntakeLevel) {
                AlgaeIntakeLevel.L2 -> ElevatorTunableValues.Heights.intakeAlgaeLowHeight.get()
                AlgaeIntakeLevel.L3 -> ElevatorTunableValues.Heights.intakeAlgaeHighHeight.get()
                else -> ElevatorTunableValues.Heights.idleHeight.get()
              }
            )

          if (elevator.clearsRobot) {
            arm.currentRequest =
              Request.ArmRequest.ClosedLoop(
                when (algaeIntakeLevel) {
                  AlgaeIntakeLevel.L2 -> ArmTunableValues.Angles.algaeLowIntakeAngle.get()
                  AlgaeIntakeLevel.L3 -> ArmTunableValues.Angles.algaeHighIntakeAngle.get()
                  else -> ArmTunableValues.Angles.idleAngle.get()
                }
              )
          }
        }

        if (RobotBase.isReal() &&
          armRollers.hasAlgae &&
          Clock.fpgaTime - lastTransitionTime > ArmRollersConstants.ALGAE_DETECTION_THRESHOLD ||
          RobotBase.isSimulation() && overrideFlagForSim
        ) {
          theoreticalGamePieceArm = GamePiece.ALGAE
        }

        if (currentRequest is SuperstructureRequest.Eject) {
          nextState = SuperstructureStates.EJECT
        } else if (currentRequest is SuperstructureRequest.Idle ||
          arm.isAtTargetedPosition && theoreticalGamePieceArm == GamePiece.ALGAE
        ) {
          currentRequest = SuperstructureRequest.Idle()
          nextState = SuperstructureStates.CLEANUP_INTAKE_ALGAE
        }
      }
      SuperstructureStates.CLEANUP_INTAKE_ALGAE -> {
        nextState = SuperstructureStates.IDLE
      }
      SuperstructureStates.CLIMB_EXTEND -> { // for getting climb set-up (straight out)
        elevator.currentRequest =
          Request.ElevatorRequest.ClosedLoop(ElevatorConstants.HEIGHTS.CLIMB_HEIGHT)
        arm.currentRequest = Request.ArmRequest.ClosedLoop(ArmConstants.ANGLES.CLIMB_ANGLE)

        climber.currentRequest =
          Request.ClimberRequest.OpenLoop(
            if (!climber.isFullyExtended) ClimberConstants.CLIMB_EXTEND_VOLTAGE else 0.0.volts,
            ClimberConstants.ROLLERS.CLASP_VOLTAGE
          )

        if (RobotBase.isReal() &&
          climber.inputs.rollersStatorCurrent >
          ClimberConstants.ROLLERS.THRESHOLD_CURRENT_LIMIT &&
          elevator.isAtTargetedPosition &&
          arm.isAtTargetedPosition
        ) {
          nextState = SuperstructureStates.CLIMB_RETRACT
        }

        when (currentRequest) {
          is SuperstructureRequest.Idle -> nextState = SuperstructureStates.IDLE
          is SuperstructureRequest.RetractClimb ->
            if (elevator.isAtTargetedPosition && arm.isAtTargetedPosition)
              nextState = SuperstructureStates.CLIMB_RETRACT
        }
      }
      SuperstructureStates.CLIMB_RETRACT -> { // for actually CLIMBING (retracting climb into robot)
        climber.currentRequest =
          Request.ClimberRequest.OpenLoop(
            if (climber.isFullyClimbed) 0.0.volts else ClimberConstants.CLIMB_RETRACT_VOLTAGE,
            ClimberConstants.ROLLERS.CLASP_VOLTAGE
          )

        // note(nathan): so... we're stuck here... heh...
      }
      SuperstructureStates.PREP_SCORE_CORAL -> {
        // if moving from one prep level to another
        if (coralScoringLevel != lastPrepLevel && lastPrepLevel != CoralLevel.NONE) {
          arm.currentRequest =
            Request.ArmRequest.ClosedLoop(
              ArmTunableValues.Angles.movingBetweenReefLevelsAngles.get()
            )
          if (arm.isAtTargetedPosition) {
            elevator.currentRequest =
              Request.ElevatorRequest.ClosedLoop(
                when (coralScoringLevel) {
                  CoralLevel.L1 -> ElevatorTunableValues.Heights.L1Height.get()
                  CoralLevel.L2 -> ElevatorTunableValues.Heights.L2Height.get()
                  CoralLevel.L3 -> ElevatorTunableValues.Heights.L3Height.get()
                  CoralLevel.L4 -> ElevatorTunableValues.Heights.L4Height.get()
                  else -> ElevatorTunableValues.Heights.idleHeight.get()
                }
              )
            if (elevator.isAtTargetedPosition) lastPrepLevel = coralScoringLevel
          }
        } else {
          if (coralScoringLevel == CoralLevel.L1 && !elevator.isAtTargetedPosition) {
            elevator.currentRequest =
              Request.ElevatorRequest.ClosedLoop(ElevatorTunableValues.Heights.l1InitHeight.get())
          } else {
            arm.currentRequest =
              Request.ArmRequest.ClosedLoop(
                when (coralScoringLevel) {
                  CoralLevel.L1 -> ArmTunableValues.Angles.l1PrepAngle.get()
                  CoralLevel.L2 -> ArmTunableValues.Angles.l2PrepAngle.get()
                  CoralLevel.L3 -> ArmTunableValues.Angles.l3PrepAngle.get()
                  CoralLevel.L4 -> ArmTunableValues.Angles.l4PrepAngle.get()
                  else -> ArmTunableValues.Angles.idleCoralAngle.get()
                }
              )

            // if l1, l2, arm needs to move all the way to make sure it doesnt hit battery or cradle
            if (arm.inputs.armPosition >= ArmConstants.ANGLES.ARM_GUARENTEED_OVER_BATTERY ||
              !(coralScoringLevel == CoralLevel.L1 || coralScoringLevel == CoralLevel.L2)
            ) {
              elevator.currentRequest =
                Request.ElevatorRequest.ClosedLoop(
                  when (coralScoringLevel) {
                    CoralLevel.L1 -> ElevatorTunableValues.Heights.L1Height.get()
                    CoralLevel.L2 -> ElevatorTunableValues.Heights.L2Height.get()
                    CoralLevel.L3 -> ElevatorTunableValues.Heights.L3Height.get()
                    CoralLevel.L4 -> ElevatorTunableValues.Heights.L4Height.get()
                    else -> ElevatorTunableValues.Heights.idleHeight.get()
                  }
                )
            }
            lastPrepLevel = coralScoringLevel
          }
        }

        when (currentRequest) {
          is SuperstructureRequest.Score -> {
            if (elevator.isAtTargetedPosition && arm.isAtTargetedPosition)
              nextState = SuperstructureStates.SCORE_CORAL
          }
          is SuperstructureRequest.Idle -> {
            nextState = SuperstructureStates.CLEANUP_SCORE_CORAL
          }
        }
      }
      SuperstructureStates.SCORE_CORAL -> {
        when (coralScoringLevel) {
          CoralLevel.L1 -> {
            // do NOT move arm here...
            armRollers.currentRequest =
              ArmRollersRequest.OpenLoop(ArmRollersConstants.OUTTAKE_CORAL_VOLTAGE)

            if (currentRequest is SuperstructureRequest.Idle) {
              nextState = SuperstructureStates.CLEANUP_SCORE_CORAL
            }

            if (Clock.fpgaTime - lastTransitionTime >=
              ArmRollersConstants.GAMEPIECE_SPITOUT_THRESHOLD
            ) {
              theoreticalGamePieceArm = GamePiece.NONE
              nextState = SuperstructureStates.CLEANUP_SCORE_CORAL
            }
          }
          else -> {
            // arm only should move a little down; if we went all the way it would hit trough in l2
            armRollers.currentRequest =
              ArmRollersRequest.OpenLoop(ArmRollersConstants.OUTTAKE_CORAL_VOLTAGE)
            arm.currentRequest =
              Request.ArmRequest.ClosedLoop(
                when (coralScoringLevel) {
                  CoralLevel.L2 -> ArmTunableValues.Angles.l2PrepAngle.get()
                  CoralLevel.L3 -> ArmTunableValues.Angles.l3PrepAngle.get()
                  CoralLevel.L4 -> ArmTunableValues.Angles.l4PrepAngle.get()
                  else -> ArmTunableValues.Angles.idleAngle.get()
                } - ArmTunableValues.Angles.scoreOffset.get()
              )

            if (currentRequest is SuperstructureRequest.Idle) {
              nextState = SuperstructureStates.CLEANUP_SCORE_CORAL
            }
            if (arm.isAtTargetedPosition &&
              Clock.fpgaTime - lastTransitionTime >=
              ArmRollersConstants.GAMEPIECE_SPITOUT_THRESHOLD
            ) {
              theoreticalGamePieceArm = GamePiece.NONE
              nextState = SuperstructureStates.CLEANUP_SCORE_CORAL
            }
          }
        }
      }
      SuperstructureStates.CLEANUP_SCORE_CORAL -> {
        when (coralScoringLevel) {
          CoralLevel.L1, CoralLevel.L2 -> {
            // if arm went straight down now, it'd hit the trough. raise elevator
            elevator.currentRequest =
              Request.ElevatorRequest.ClosedLoop(
                if (theoreticalGamePieceArm == GamePiece.NONE)
                  ElevatorTunableValues.Heights.idleHeight.get()
                else ElevatorTunableValues.Heights.idleCoralHeight.get()
              )
            if (elevator.isAtTargetedPosition) {
              arm.currentRequest =
                Request.ArmRequest.ClosedLoop(ArmTunableValues.Angles.idleAngle.get())
              // now that arm is in a safe spot, idle will deal with the rest of the transition
              if (arm.isAtTargetedPosition) nextState = SuperstructureStates.IDLE
            }
          }
          else -> {
            arm.currentRequest =
              Request.ArmRequest.ClosedLoop(ArmTunableValues.Angles.idleAngle.get())
            // now that arm is in a safe spot, idle will deal with the rest of the transition
            if (arm.isAtTargetedPosition) nextState = SuperstructureStates.IDLE
          }
        }
        lastPrepLevel = CoralLevel.NONE
      }
      SuperstructureStates.PREP_SCORE_ALGAE -> {
        elevator.currentRequest =
          Request.ElevatorRequest.ClosedLoop(
            when (algaeScoringLevel) {
              AlgaeScoringLevel.PROCESSOR -> ElevatorTunableValues.Heights.processorHeight.get()
              AlgaeScoringLevel.BARGE -> ElevatorTunableValues.Heights.bargeHeight.get()
              else -> ElevatorTunableValues.Heights.idleHeight.get()
            }
          )

        arm.currentRequest =
          Request.ArmRequest.ClosedLoop(
            when (algaeScoringLevel) {
              AlgaeScoringLevel.PROCESSOR -> ArmTunableValues.Angles.processorAngle.get()
              AlgaeScoringLevel.BARGE -> ArmTunableValues.Angles.bargeAngle.get()
              else -> ArmTunableValues.Angles.idleCoralAngle.get()
            }
          )

        when (currentRequest) {
          is SuperstructureRequest.Idle -> nextState = SuperstructureStates.IDLE
          is SuperstructureRequest.Score -> {
            if (elevator.isAtTargetedPosition && arm.isAtTargetedPosition)
              nextState = SuperstructureStates.SCORE_ALGAE
          }
        }
      }
      SuperstructureStates.SCORE_ALGAE -> {
        armRollers.currentRequest =
          ArmRollersRequest.OpenLoop(ArmRollersConstants.OUTTAKE_ALGAE_VOLTAGE)

        if (currentRequest is SuperstructureRequest.Idle) {
          nextState = SuperstructureStates.CLEANUP_SCORE_ALGAE
        }

        if (Clock.fpgaTime - lastTransitionTime >=
          ArmRollersConstants.GAMEPIECE_SPITOUT_THRESHOLD
        ) {
          theoreticalGamePieceArm = GamePiece.NONE
          nextState = SuperstructureStates.CLEANUP_SCORE_ALGAE
        }
      }
      SuperstructureStates.CLEANUP_SCORE_ALGAE -> {
        nextState = SuperstructureStates.IDLE
      }
      SuperstructureStates.EJECT -> {
        lastPrepLevel = CoralLevel.NONE

        intake.currentRequest =
          Request.IntakeRequest.TargetingPosition(
            IntakeTunableValues.idlePosition.get(),
            IntakeTunableValues.ejectRollerVoltage.get()
          )
        indexer.currentRequest = Request.IndexerRequest.Eject()

        if (!elevator.clearsRobot) {
          elevator.currentRequest =
            Request.ElevatorRequest.ClosedLoop(ElevatorConstants.HEIGHTS.CLEARS_ROBOT)
        } else {
          arm.currentRequest = Request.ArmRequest.ClosedLoop(ArmConstants.ANGLES.EJECT_ANGLE)
          if (arm.isAtTargetedPosition) {
            armRollers.currentRequest =
              ArmRollersRequest.OpenLoop(ArmRollersConstants.EJECT_VOLTAGE)
          }
        }

        if (currentRequest is SuperstructureRequest.Idle) {
          theoreticalGamePieceArm = GamePiece.NONE
          nextState = SuperstructureStates.IDLE
        }
      }
    }

    if (nextState != currentState) lastTransitionTime = Clock.fpgaTime

    currentState = nextState
  }

  fun requestIdleCommand(): Command {
    val returnCommand =
      run { currentRequest = SuperstructureRequest.Idle() }.until {
        isAtRequestedState && currentState == SuperstructureStates.IDLE
      }
    returnCommand.name = "RequestIdleCommand"
    return returnCommand
  }

  fun ejectCommand(): Command {
    val returnCommand =
      run { currentRequest = SuperstructureRequest.Eject() }.until {
        isAtRequestedState && currentState == SuperstructureStates.EJECT
      }
    returnCommand.name = "EjectCommand"
    return returnCommand
  }

  // -------------------------------- Redirect Commands --------------------------------
  fun prepL1OrAlgaeGroundCommand(): Command {
    return ConditionalCommand(
      prepScoreCoralCommand(CoralLevel.L1), intakeAlgaeCommand(AlgaeIntakeLevel.GROUND)
    ) {
      theoreticalGamePieceArm == GamePiece.CORAL
    }
  }

  fun prepL2OrProcessorCommand(): Command {
    return ConditionalCommand(
      prepScoreCoralCommand(CoralLevel.L2), prepScoreAlgaeCommand(AlgaeScoringLevel.PROCESSOR)
    ) {
      theoreticalGamePieceArm == GamePiece.CORAL
    }
  }

  fun prepL3OrAlgaeReefCommand(): Command {
    return ConditionalCommand(
      prepScoreCoralCommand(CoralLevel.L3),
      ConditionalCommand(
        intakeAlgaeCommand(AlgaeIntakeLevel.L3), intakeAlgaeCommand(AlgaeIntakeLevel.L2)
      ) {
        vision.lastTrigVisionUpdate.targetTagID in Constants.Universal.HIGH_ALGAE_REEF_TAGS
      }
    ) { theoreticalGamePieceArm == GamePiece.CORAL }
  }

  fun prepL4OrBargeCommand(): Command {
    return ConditionalCommand(
      prepScoreCoralCommand(CoralLevel.L4), prepScoreAlgaeCommand(AlgaeScoringLevel.BARGE)
    ) {
      theoreticalGamePieceArm == GamePiece.CORAL
    }
  }

  // --------------------------------- Intake Commands ---------------------------------
  fun intakeCoralCommand(): Command {
    val returnCommand =
      runOnce { currentRequest = SuperstructureRequest.IntakeCoral() }.until {
        isAtRequestedState && currentState == SuperstructureStates.IDLE
      }
    returnCommand.name = "IntakeCoral"
    return returnCommand
  }

  fun intakeAlgaeCommand(level: AlgaeIntakeLevel): Command {
    val returnCommand =
      runOnce { currentRequest = SuperstructureRequest.IntakeAlgae(level) }.until {
        isAtRequestedState && currentState == SuperstructureStates.INTAKE_ALGAE
      }
    returnCommand.name = "IntakeAlgae"
    return returnCommand
  }

  // -------------------------------- Prep Score Commands --------------------------------
  fun prepScoreCoralCommand(level: CoralLevel): Command {
    val returnCommand =
      runOnce { currentRequest = SuperstructureRequest.PrepScoreCoral(level) }.until {
        isAtRequestedState && currentState == SuperstructureStates.PREP_SCORE_CORAL
      }
    returnCommand.name = "PrepScoreCoralCommand"
    return returnCommand
  }

  fun prepScoreAlgaeCommand(level: AlgaeScoringLevel): Command {
    val returnCommand =
      runOnce { currentRequest = SuperstructureRequest.PrepScoreAlgae(level) }.until {
        isAtRequestedState && currentState == SuperstructureStates.PREP_SCORE_ALGAE
      }
    returnCommand.name = "PrepScoreAlgaeCommand"
    return returnCommand
  }

  // -------------------------------- Score Commands --------------------------------
  fun scoreCommand(): Command {
    val returnCommand =
      runOnce { currentRequest = SuperstructureRequest.Score() }.until {
        isAtRequestedState &&
          (
            currentState == SuperstructureStates.SCORE_CORAL ||
              currentState == SuperstructureStates.SCORE_ALGAE
            )
      }
    returnCommand.name = "ScoreCommand"
    return returnCommand
  }

  // -------------------------------- Climb Commands --------------------------------
  fun climbExtendCommand(): Command {
    val returnCommand =
      runOnce { currentRequest = SuperstructureRequest.ExtendClimb() }.until {
        isAtRequestedState && currentState == SuperstructureStates.CLIMB_EXTEND
      }

    returnCommand.name = "ClimbExtendCommand"
    return returnCommand
  }

  fun climbRetractCommand(): Command {
    val returnCommand =
      runOnce { currentRequest = SuperstructureRequest.RetractClimb() }.until {
        isAtRequestedState && currentState == SuperstructureStates.CLIMB_RETRACT
      }

    returnCommand.name = "ClimbRetractCommand"
    return returnCommand
  }

  // -------------------------------- Test Commands --------------------------------
  fun overrideFlag(setOverride: Boolean): Command {
    val returnCommand = runOnce { if (RobotBase.isSimulation()) overrideFlagForSim = setOverride }

    returnCommand.name = "OverrideFlagCommand"
    return returnCommand
  }

  // -------------------------------- Gamepiece Reset--------------------------------

  fun resetGamepieceCommand(gamePiece: GamePiece): Command {
    val returnCommand = runOnce { theoreticalGamePieceArm = gamePiece }
    returnCommand.name = "ResetGamepieceCommand"
    return returnCommand
  }

  companion object {
    enum class SuperstructureStates {
      UNINITIALIZED,
      TUNING,
      HOME_PREP,
      HOME,
      IDLE,
      GROUND_INTAKE_CORAL,
      GROUND_INTAKE_CORAL_CLEANUP,
      INTAKE_CORAL_INTO_ARM,
      INTAKE_ALGAE,
      CLEANUP_INTAKE_ALGAE,
      CLIMB_EXTEND,
      CLIMB_RETRACT,
      PREP_SCORE_CORAL,
      SCORE_CORAL,
      CLEANUP_SCORE_CORAL,
      PREP_SCORE_ALGAE,
      SCORE_ALGAE,
      CLEANUP_SCORE_ALGAE,
      EJECT
    }
  }
}
