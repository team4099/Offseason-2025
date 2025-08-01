package com.team4099.robot2025.subsystems.superstructure

import com.team4099.lib.hal.Clock
import com.team4099.robot2025.config.constants.ArmConstants
import com.team4099.robot2025.config.constants.ClimberConstants
import com.team4099.robot2025.config.constants.ElevatorConstants
import com.team4099.robot2025.config.constants.IntakeConstants
import com.team4099.robot2025.subsystems.Arm.Arm
import com.team4099.robot2025.subsystems.Arm.ArmTunableValues
import com.team4099.robot2025.subsystems.climber.Climber
import com.team4099.robot2025.subsystems.drivetrain.drive.Drivetrain
import com.team4099.robot2025.subsystems.elevator.Elevator
import com.team4099.robot2025.subsystems.elevator.ElevatorTunableValues
import com.team4099.robot2025.subsystems.indexer.Indexer
import com.team4099.robot2025.subsystems.intake.Intake
import com.team4099.robot2025.subsystems.intake.IntakeTunableValues
import com.team4099.robot2025.subsystems.limelight.LimelightVision
import com.team4099.robot2025.subsystems.vision.Vision
import com.team4099.robot2025.util.CustomLogger
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.team4099.lib.units.base.inches
import org.team4099.lib.units.derived.volts
import com.team4099.robot2025.config.constants.Constants.Universal.AlgaeIntakeLevel as AlgaeIntakeLevel
import com.team4099.robot2025.config.constants.Constants.Universal.AlgaeScoringLevel as AlgaeScoringLevel
import com.team4099.robot2025.config.constants.Constants.Universal.CoralLevel as CoralLevel
import com.team4099.robot2025.config.constants.Constants.Universal.GamePiece as GamePiece
import com.team4099.robot2025.config.constants.RollersConstants as ArmRollersConstants
import com.team4099.robot2025.subsystems.Arm.Rollers.Rollers as ArmRollers
import com.team4099.robot2025.subsystems.superstructure.Request.RollersRequest as ArmRollersRequest

class Superstructure(
  private val drivetrain: Drivetrain,
  private val vision: Vision,
  private val limelight: LimelightVision,
  private val elevator: Elevator,
  private val arm: Arm,
  private val armRollers: ArmRollers,
  private val climber: Climber,
  private val intake: Intake,
  private val indexer: Indexer
) : SubsystemBase() {

  var theoreticalGamePieceArm: GamePiece = GamePiece.CORAL // preload !!
  var theoreticalGamePieceHardstop: GamePiece = GamePiece.NONE

  //    get() {
  //      if (rollers.hasCoral || ramp.hasCoral) {
  //        return GamePiece.CORAL
  //      } else {
  //        return field
  //      }
  //    } add this later :)

  private var lastCoralScoringLevel: CoralLevel = CoralLevel.NONE
  private var coralScoringLevel: CoralLevel = CoralLevel.NONE

  private var lastAlgaeIntakeLevel: AlgaeIntakeLevel = AlgaeIntakeLevel.NONE
  private var algaeIntakeLevel: AlgaeIntakeLevel = AlgaeIntakeLevel.NONE

  private var lastAlgaeScoringLevel: AlgaeScoringLevel = AlgaeScoringLevel.NONE
  private var algaeScoringLevel: AlgaeScoringLevel = AlgaeScoringLevel.NONE

  var currentRequest: Request.SuperstructureRequest = Request.SuperstructureRequest.Idle()
    set(value) {
      when (value) {
        is Request.SuperstructureRequest.IntakeAlgae -> {
          algaeIntakeLevel = value.level
        }
        is Request.SuperstructureRequest.PrepScoreCoral -> {
          coralScoringLevel = value.level
        }
        is Request.SuperstructureRequest.PrepScoreAlgae -> {
          algaeScoringLevel = value.level
        }
        is Request.SuperstructureRequest.Score -> {
          coralScoringLevel = coralScoringLevel
          algaeScoringLevel = algaeScoringLevel
        }
        else -> {
          coralScoringLevel = CoralLevel.NONE
        }
      }
      field = value
    }

  var currentState: SuperstructureStates = SuperstructureStates.UNINITIALIZED

  var isAtRequestedState: Boolean = false

  private var lastTransitionTime = Clock.fpgaTime

  override fun periodic() {
    // add this later when superstructure is done :)
    //    val ledLoopStartTime = Clock.realTimestamp
    //    leds.periodic()
    //    CustomLogger.recordOutput(
    //      "LoggedRobot/Subsystems/ledLoopTimeMS",
    //      (Clock.realTimestamp - ledLoopStartTime).inMilliseconds
    //    )
    //
    //    val armLoopStartTime = Clock.realTimestamp
    //    arm.periodic()
    //    CustomLogger.recordOutput(
    //      "LoggedRobot/Subsystems/ArmLoopTimeMS",
    //      (Clock.realTimestamp - armLoopStartTime).inMilliseconds
    //    )
    //
    //    val climberLoopStartTime = Clock.realTimestamp
    //    climber.periodic()
    //    CustomLogger.recordOutput(
    //      "LoggedRobot/Subsystems/ClimberLoopTimeMS",
    //      (Clock.realTimestamp - climberLoopStartTime).inMilliseconds
    //    )
    //
    //    val elevatorLoopStartTime = Clock.realTimestamp
    //    elevator.periodic()
    //    CustomLogger.recordOutput(
    //      "LoggedRobot/Subsystems/ElevatorLoopTimeMS",
    //      (Clock.realTimestamp - elevatorLoopStartTime).inMilliseconds
    //    )
    //
    //    val rollersLoopStartTime = Clock.realTimestamp
    //    rollers.periodic()
    //    CustomLogger.recordOutput(
    //      "LoggedRobot/Subsystems/RollersLoopTimeMS",
    //      (Clock.realTimestamp - rollersLoopStartTime).inMilliseconds
    //    )
    //
    //    val rampLoopStartTime = Clock.realTimestamp
    //    ramp.periodic()
    //    CustomLogger.recordOutput(
    //      "LoggedRobot/Subsystems/RampLoopTimeMS",
    //      (Clock.realTimestamp - rampLoopStartTime).inMilliseconds
    //    )

    CustomLogger.recordOutput("Superstructure/currentRequest", currentRequest.javaClass.simpleName)
    CustomLogger.recordOutput("Superstructure/currentState", currentState.name)
    CustomLogger.recordOutput("Superstructure/isAtAllTargetedPositions", isAtRequestedState)
    CustomLogger.recordOutput("Superstructure/theoreticalGamePieceArm", theoreticalGamePieceArm)
    CustomLogger.recordOutput(
      "Superstructure/theoreticalGamePieceHardstop", theoreticalGamePieceHardstop
    )

    var nextState = currentState
    when (currentState) {
      // General States
      SuperstructureStates.UNINITIALIZED -> {
        nextState = SuperstructureStates.HOME_PREP
      }

      SuperstructureStates.HOME_PREP ->{
      arm.currentRequest = Request.ArmRequest.ClosedLoop(ArmConstants.ANGLES.HOME_ANGLE)
        if (arm.isAtTargetedPosition){
          nextState = SuperstructureStates.HOME
        }
      }

      SuperstructureStates.HOME -> {
        elevator.currentRequest = Request.ElevatorRequest.Home()

        if (elevator.isHomed) {
          elevator.currentRequest =
            Request.ElevatorRequest.ClosedLoop(
              if (theoreticalGamePieceArm == GamePiece.CORAL)
                ElevatorTunableValues.Heights.idleCoralHeight.get()
              else ElevatorTunableValues.Heights.idleHeight.get()
            )
          if (elevator.isAtTargetedPosition) nextState = SuperstructureStates.IDLE
        }
      }
      SuperstructureStates.IDLE -> {
        climber.currentRequest = Request.ClimberRequest.OpenLoop(0.0.volts, 0.0.volts)
        intake.currentRequest =
          Request.IntakeRequest.TargetingPosition(
            IntakeTunableValues.idlePosition.get(), 0.0.volts
          )

        when (theoreticalGamePieceArm) {
          GamePiece.CORAL -> {
            elevator.currentRequest =
              Request.ElevatorRequest.ClosedLoop(
                ElevatorTunableValues.Heights.idleCoralHeight.get()
              )

            // since we (likely) just intaked coral, make sure the elevator moves out of the way of
            // cradle before arm moves into its idle coral position
            if (elevator.clearsBattery) {
              arm.currentRequest =
                Request.ArmRequest.ClosedLoop(ArmTunableValues.Angles.idleCoralAngle.get())
            }

            armRollers.currentRequest =
              Request.RollersRequest.OpenLoop(ArmRollersConstants.IDLE_CORAL_VOLTAGE)
          }
          GamePiece.ALGAE -> {
            // arm should move first in case elevator ends up moving down
            arm.currentRequest =
              Request.ArmRequest.ClosedLoop(ArmTunableValues.Angles.idleAlgaeAngle.get())
            armRollers.currentRequest =
              Request.RollersRequest.OpenLoop(ArmRollersConstants.IDLE_ALGAE_VOLTAGE)
            if (arm.isAtTargetedPosition) {
              elevator.currentRequest =
                Request.ElevatorRequest.ClosedLoop(
                  ElevatorTunableValues.Heights.idleAlgaeHeight.get()
                )
            }
          }
          GamePiece.NONE -> {
            // since we just scored, make sure arm can retract before elevator pulls it down
            // so it doesnt hit the battery
            arm.currentRequest = Request.ArmRequest.ClosedLoop(ArmConstants.ANGLES.IDLE_ANGLE)
            armRollers.currentRequest =
              Request.RollersRequest.OpenLoop(ArmRollersConstants.IDLE_VOLTAGE)
            if (arm.isAtTargetedPosition) {
              elevator.currentRequest =
                Request.ElevatorRequest.ClosedLoop(ElevatorTunableValues.Heights.idleHeight.get())
            }
          }
        }

        // idle to request transitions
        nextState =
          // if you were holding a coral but dropped it
          if (theoreticalGamePieceArm != GamePiece.ALGAE &&
            theoreticalGamePieceHardstop == GamePiece.CORAL
          )
            SuperstructureStates.INTAKE_CORAL_INTO_ARM
          else
            when (currentRequest) {
              is Request.SuperstructureRequest.Home -> SuperstructureStates.HOME
              is Request.SuperstructureRequest.IntakeCoral ->
                SuperstructureStates.GROUND_INTAKE_CORAL
              is Request.SuperstructureRequest.IntakeAlgae -> SuperstructureStates.INTAKE_ALGAE
              is Request.SuperstructureRequest.PrepScoreCoral ->
                SuperstructureStates.PREP_SCORE_CORAL
              is Request.SuperstructureRequest.PrepScoreAlgae ->
                SuperstructureStates.PREP_SCORE_ALGAE
              is Request.SuperstructureRequest.ExtendClimb -> SuperstructureStates.CLIMB_EXTEND
              is Request.SuperstructureRequest.RetractClimb ->
                SuperstructureStates.CLIMB_RETRACT
              else -> currentState
            }
      }
      SuperstructureStates.GROUND_INTAKE_CORAL -> {
        intake.currentRequest =
          Request.IntakeRequest.TargetingPosition(
            IntakeTunableValues.coralPosition.get(), IntakeConstants.Rollers.INTAKE_VOLTAGE
          )
        indexer.currentRequest = Request.IndexerRequest.Index()

        if (theoreticalGamePieceHardstop == GamePiece.CORAL ||
          currentRequest is Request.SuperstructureRequest.Idle
        ) {
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

          if (armRollers.hasCoral) {
            theoreticalGamePieceArm = GamePiece.CORAL
            theoreticalGamePieceHardstop = GamePiece.NONE
          }
        }

        if (currentRequest is Request.SuperstructureRequest.Idle ||
          arm.isAtTargetedPosition && theoreticalGamePieceArm == GamePiece.CORAL
        ) {
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
          if (!elevator.clearsBattery &&
            arm.inputs.armPosition <= ArmConstants.ANGLES.ARM_GUARENTEED_OVER_BATTERY
          ) {
            elevator.currentRequest =
              Request.ElevatorRequest.ClosedLoop(
                ElevatorConstants.HEIGHTS.ELEVATOR_HEIGHT_TO_CLEAR_ARM + 2.0.inches
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

          if (elevator.clearsBattery) {
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

        if (armRollers.hasAlgae) {
          theoreticalGamePieceArm = GamePiece.ALGAE
        }

        if (currentRequest is Request.SuperstructureRequest.Idle ||
          arm.isAtTargetedPosition && theoreticalGamePieceArm == GamePiece.ALGAE
        ) {
          nextState = SuperstructureStates.CLEANUP_INTAKE_ALGAE
        }
      }
      SuperstructureStates.CLEANUP_INTAKE_ALGAE -> {
        if (theoreticalGamePieceArm != GamePiece.ALGAE && !elevator.clearsBattery) {
          elevator.currentRequest =
            Request.ElevatorRequest.ClosedLoop(
              ElevatorConstants.HEIGHTS.ELEVATOR_HEIGHT_TO_CLEAR_ARM + 2.0.inches
            )
        } else {
          // case 1 - holding algae, idle will deal with the transition nicely
          // case 2 - we've gone up enough in height to safely transition to idle
          nextState = SuperstructureStates.IDLE
        }
      }
      SuperstructureStates.CLIMB_EXTEND -> { // for getting climb set-up (straight out)
        climber.currentRequest =
          Request.ClimberRequest.ClosedLoop(ClimberConstants.FULLY_EXTENDED_ANGLE)

        if (climber.isAtTargetedPosition) {
          nextState = SuperstructureStates.IDLE
        }

        when (currentRequest) {
          is Request.SuperstructureRequest.Idle -> SuperstructureStates.IDLE
          is Request.SuperstructureRequest.RetractClimb -> SuperstructureStates.CLIMB_RETRACT
        }
      }
      SuperstructureStates.CLIMB_RETRACT -> { // for actually CLIMBING (retracting climb into robot)
        climber.currentRequest =
          if (climber.isAtTargetedPosition) {
            Request.ClimberRequest.OpenLoop(
              0.0.volts, // don't keep open looping or you'll stall out the motor
              ClimberConstants.INTAKE_CAGE_VOLTAGE
            )
          } else {
            Request.ClimberRequest.OpenLoop(
              ClimberConstants.CLIMB_RETRACT_VOLTAGE, ClimberConstants.INTAKE_CAGE_VOLTAGE
            )
          }

        when (currentRequest) {
          is Request.SuperstructureRequest.Idle -> SuperstructureStates.IDLE
          is Request.SuperstructureRequest.ExtendClimb -> SuperstructureStates.CLIMB_EXTEND
        }
      }
      SuperstructureStates.PREP_SCORE_CORAL -> {
        elevator.currentRequest =
          when (coralScoringLevel) {
            CoralLevel.L1 ->
              Request.ElevatorRequest.ClosedLoop(ElevatorTunableValues.Heights.L1Height.get())
            CoralLevel.L2 ->
              Request.ElevatorRequest.ClosedLoop(ElevatorTunableValues.Heights.L2Height.get())
            CoralLevel.L3 ->
              Request.ElevatorRequest.ClosedLoop(ElevatorTunableValues.Heights.L3Height.get())
            CoralLevel.L4 ->
              Request.ElevatorRequest.ClosedLoop(ElevatorTunableValues.Heights.L4Height.get())
            else ->
              Request.ElevatorRequest.ClosedLoop(ElevatorTunableValues.Heights.idleHeight.get())
          }

        if (elevator.clearsBattery) {
          arm.currentRequest =
            when (coralScoringLevel) {
              CoralLevel.L1 ->
                Request.ArmRequest.ClosedLoop(
                  ArmTunableValues.Angles.l1PrepAngle.get(),
                )
              CoralLevel.L2 ->
                Request.ArmRequest.ClosedLoop(
                  ArmTunableValues.Angles.l2PrepAngle.get(),
                )
              CoralLevel.L3 ->
                Request.ArmRequest.ClosedLoop(
                  ArmTunableValues.Angles.l3PrepAngle.get(),
                )
              CoralLevel.L4 ->
                Request.ArmRequest.ClosedLoop(
                  ArmTunableValues.Angles.l4PrepAngle.get(),
                )
              else -> Request.ArmRequest.ClosedLoop(ArmTunableValues.Angles.idleCoralAngle.get())
            }
        }

        when (currentRequest) {
          is Request.SuperstructureRequest.Score -> {
            if (elevator.isAtTargetedPosition && arm.isAtTargetedPosition)
              nextState = SuperstructureStates.SCORE_CORAL
          }
          is Request.SuperstructureRequest.Idle -> {
            nextState = SuperstructureStates.IDLE
          }
        }
      }
      SuperstructureStates.SCORE_CORAL -> {
        when (coralScoringLevel) {
          CoralLevel.L1 -> {
            // do NOT move arm here...
            armRollers.currentRequest =
              ArmRollersRequest.OpenLoop(ArmRollersConstants.OUTTAKE_CORAL_VOLTAGE)

            if (currentRequest is Request.SuperstructureRequest.Idle) {
              nextState = SuperstructureStates.CLEANUP_SCORE_CORAL
            }

            if (Clock.fpgaTime - lastTransitionTime >=
              ArmRollersConstants.GAMEPIECE_SPITOUT_THRESHOLD
            ) {
              theoreticalGamePieceArm = GamePiece.NONE
              nextState = SuperstructureStates.CLEANUP_SCORE_CORAL
            }
          }
          CoralLevel.L2, CoralLevel.L3, CoralLevel.L4 -> {
            // arm only should move a little down; if we went all the way it would hit trough in l2
            armRollers.currentRequest =
              ArmRollersRequest.OpenLoop(ArmRollersConstants.OUTTAKE_CORAL_VOLTAGE)
            arm.currentRequest =
              Request.ArmRequest.ClosedLoop(
                when (coralScoringLevel) {
                  CoralLevel.L2 ->
                    ArmTunableValues.Angles.l2PrepAngle.get() -
                      ArmTunableValues.Angles.scoreOffset.get()
                  CoralLevel.L3 ->
                    ArmTunableValues.Angles.l3PrepAngle.get() -
                      ArmTunableValues.Angles.scoreOffset.get()
                  CoralLevel.L4 ->
                    ArmTunableValues.Angles.l4PrepAngle.get() -
                      ArmTunableValues.Angles.scoreOffset.get()
                  else -> ArmTunableValues.Angles.idleAngle.get()
                } - ArmTunableValues.Angles.scoreOffset.get()
              )

            if (currentRequest is Request.SuperstructureRequest.Idle) {
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
          else -> {
            // todo note(nathan): idek what to put here tbh
            nextState = SuperstructureStates.CLEANUP_SCORE_CORAL
          }
        }
      }
      SuperstructureStates.CLEANUP_SCORE_CORAL -> {
        when (coralScoringLevel) {
          CoralLevel.L1, CoralLevel.L2 -> {
            // if arm went straight down now, it'd hit the trough. raise elevator
            elevator.currentRequest =
              Request.ElevatorRequest.ClosedLoop(ElevatorConstants.HEIGHTS.LOW_SCORE_OFFSET)
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
      }
      SuperstructureStates.PREP_SCORE_ALGAE -> {
        elevator.currentRequest =
          when (algaeScoringLevel) {
            AlgaeScoringLevel.PROCESSOR ->
              Request.ElevatorRequest.ClosedLoop(
                ElevatorTunableValues.Heights.processorHeight.get()
              )
            AlgaeScoringLevel.BARGE ->
              Request.ElevatorRequest.ClosedLoop(
                ElevatorTunableValues.Heights.bargeHeight.get()
              )
            else ->
              Request.ElevatorRequest.ClosedLoop(ElevatorTunableValues.Heights.idleHeight.get())
          }

        if (elevator.clearsBattery) {
          arm.currentRequest =
            when (algaeScoringLevel) {
              AlgaeScoringLevel.PROCESSOR ->
                Request.ArmRequest.ClosedLoop(ArmTunableValues.Angles.processorAngle.get())
              AlgaeScoringLevel.BARGE ->
                Request.ArmRequest.ClosedLoop(ArmTunableValues.Angles.bargeAngle.get())
              else -> Request.ArmRequest.ClosedLoop(ArmTunableValues.Angles.idleCoralAngle.get())
            }
        }

        when (currentRequest) {
          is Request.SuperstructureRequest.Idle -> nextState = SuperstructureStates.IDLE
          is Request.SuperstructureRequest.Score -> {
            if (elevator.isAtTargetedPosition && arm.isAtTargetedPosition)
              nextState = SuperstructureStates.SCORE_ALGAE
          }
        }
      }
      SuperstructureStates.SCORE_ALGAE -> {
        armRollers.currentRequest =
          ArmRollersRequest.OpenLoop(ArmRollersConstants.OUTTAKE_ALGAE_VOLTAGE)

        if (currentRequest is Request.SuperstructureRequest.Idle) {
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
        nextState =
          if (theoreticalGamePieceArm == GamePiece.NONE &&
            theoreticalGamePieceHardstop == GamePiece.CORAL
          ) {
            // dw, intake_coral_into_arm deals with the transition
            SuperstructureStates.INTAKE_CORAL_INTO_ARM
          } else {
            // idle should deal with it
            SuperstructureStates.IDLE
          }
      }
      SuperstructureStates.EJECT -> {
        arm.currentRequest = Request.ArmRequest.ClosedLoop(ArmConstants.ANGLES.EJECT_ANGLE)
        if(arm.isAtTargetedPosition) {
        armRollers.currentRequest = Request.RollersRequest.OpenLoop(ArmRollersConstants.EJECT_VOLTAGE)
        nextState = SuperstructureStates.IDLE
        }
      }
    }

    if (nextState != currentState) lastTransitionTime = Clock.fpgaTime

    currentState = nextState
  }

  companion object {
    enum class SuperstructureStates {
      UNINITIALIZED,
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
