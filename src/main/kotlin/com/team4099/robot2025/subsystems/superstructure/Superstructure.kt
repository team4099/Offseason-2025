package com.team4099.robot2025.subsystems.superstructure

import com.team4099.lib.hal.Clock
import com.team4099.robot2025.Robot
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
import com.team4099.robot2025.subsystems.intake.Intake
import com.team4099.robot2025.subsystems.intake.IntakeTunableValues
import com.team4099.robot2025.subsystems.limelight.LimelightVision
import com.team4099.robot2025.subsystems.vision.Vision
import com.team4099.robot2025.util.CustomLogger
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.team4099.lib.units.base.seconds
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
  private val intake: Intake
) : SubsystemBase() {

  var theoreticalGamePieceArm: GamePiece = GamePiece.NONE
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
        nextState = SuperstructureStates.HOME
      }
      SuperstructureStates.GROUND_INTAKE_CORAL -> {
        intake.currentRequest =
          Request.IntakeRequest.TargetingPosition(
            IntakeTunableValues.coralPosition.get(), IntakeConstants.Rollers.INTAKE_VOLTAGE
          )

        // update with canrange

        //        if (canrange.hasCoral) {
        //          theoreticalGamePieceHardstop = GamePiece.CORAL
        //        }

        if (theoreticalGamePieceHardstop == GamePiece.CORAL ||
          currentRequest is Request.SuperstructureRequest.Idle
        ) {
          nextState = SuperstructureStates.GROUND_INTAKE_CORAL_CLEANUP
        }
      }
      SuperstructureStates.GROUND_INTAKE_CORAL_CLEANUP -> {
        if (Robot.isAutonomous) {
          nextState = SuperstructureStates.IDLE
        } else {
          intake.currentRequest =
            Request.IntakeRequest.TargetingPosition(
              IntakeTunableValues.stowPosition.get(), IntakeConstants.Rollers.IDLE_VOLTAGE
            )

          if (intake.isAtTargetedPosition) {
            if (theoreticalGamePieceArm == GamePiece.NONE &&
              theoreticalGamePieceHardstop ==
              GamePiece.CORAL
            ) { // if not holding an algae, intake it now
              nextState = SuperstructureStates.INTAKE_CORAL_INTO_ARM
            } else if (theoreticalGamePieceHardstop ==
              GamePiece.CORAL
            ) { // if holding an algae, keep in hardstop
              nextState = SuperstructureStates.IDLE
            }
          }
          if (currentRequest is Request.SuperstructureRequest.Idle) {
            nextState = SuperstructureStates.IDLE
          }
        }
      }
      SuperstructureStates.INTAKE_CORAL_INTO_ARM -> {
        arm.currentRequest =
          Request.ArmRequest.ClosedLoop(ArmTunableValues.Angles.hardstopIntakeAngle.get())
        armRollers.currentRequest =
          ArmRollersRequest.OpenLoop(ArmRollersConstants.INTAKE_CORAL_VOLTAGE)

        if (armRollers.hasCoral) {
          theoreticalGamePieceArm = GamePiece.CORAL
          theoreticalGamePieceHardstop = GamePiece.NONE
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

        elevator.currentRequest =
          when (algaeIntakeLevel) {
            AlgaeIntakeLevel.GROUND ->
              Request.ElevatorRequest.ClosedLoop(
                ElevatorTunableValues.Heights.intakeAlgaeGroundHeight.get()
              )
            AlgaeIntakeLevel.L2 ->
              Request.ElevatorRequest.ClosedLoop(
                ElevatorTunableValues.Heights.intakeAlgaeLowHeight.get()
              )
            AlgaeIntakeLevel.L3 ->
              Request.ElevatorRequest.ClosedLoop(
                ElevatorTunableValues.Heights.intakeAlgaeHighHeight.get()
              )
            else ->
              Request.ElevatorRequest.ClosedLoop(ElevatorTunableValues.Heights.idleHeight.get())
          }

        if (armRollers.hasAlgae) {
          theoreticalGamePieceArm = GamePiece.ALGAE
        }

        if (currentRequest is Request.SuperstructureRequest.Idle ||
          arm.isAtTargetedPosition && theoreticalGamePieceArm == GamePiece.ALGAE
        ) {
          nextState = SuperstructureStates.IDLE
        }
      }
      SuperstructureStates.HOME -> {
        elevator.currentRequest = Request.ElevatorRequest.Home()

        if (elevator.isHomed) {
          nextState = SuperstructureStates.IDLE
        }
      }
      SuperstructureStates.IDLE -> {
        climber.currentRequest = Request.ClimberRequest.OpenLoop(0.0.volts, 0.0.volts)
        intake.currentRequest = Request.IntakeRequest.OpenLoop(0.0.volts, 0.0.volts)

        when (theoreticalGamePieceArm) {
          GamePiece.CORAL -> {
            elevator.currentRequest =
              Request.ElevatorRequest.ClosedLoop(
                ElevatorTunableValues.Heights.idleCoralHeight.get()
              )
            arm.currentRequest =
              Request.ArmRequest.ClosedLoop(ArmTunableValues.Angles.idleCoralAngle.get())
            armRollers.currentRequest =
              Request.RollersRequest.OpenLoop(ArmRollersConstants.IDLE_CORAL_VOLTAGE)
          }
          GamePiece.ALGAE -> {
            elevator.currentRequest =
              Request.ElevatorRequest.ClosedLoop(
                ElevatorTunableValues.Heights.idleAlgaeHeight.get()
              )
            arm.currentRequest =
              Request.ArmRequest.ClosedLoop(ArmTunableValues.Angles.idleAlgaeAngle.get())
            armRollers.currentRequest =
              Request.RollersRequest.OpenLoop(ArmRollersConstants.IDLE_ALGAE_VOLTAGE)
          }
          GamePiece.NONE -> {
            elevator.currentRequest =
              Request.ElevatorRequest.ClosedLoop(ElevatorTunableValues.Heights.idleHeight.get())
            arm.currentRequest = Request.ArmRequest.ClosedLoop(ArmConstants.ANGLES.IDLE_ANGLE)
            armRollers.currentRequest =
              Request.RollersRequest.OpenLoop(ArmRollersConstants.IDLE_ALGAE_VOLTAGE)
          }
        }

        // idle to request transitions
        nextState =
          if (theoreticalGamePieceArm == GamePiece.NONE &&
            theoreticalGamePieceHardstop == GamePiece.CORAL
          )
            SuperstructureStates.INTAKE_CORAL_INTO_ARM
          else
            when (currentRequest) {
              is Request.SuperstructureRequest.Home -> SuperstructureStates.HOME
              is Request.SuperstructureRequest.PrepScoreCoral ->
                SuperstructureStates.SCORE_PREP_CORAL
              is Request.SuperstructureRequest.PrepScoreAlgae ->
                SuperstructureStates.PREP_SCORE_ALGAE
              is Request.SuperstructureRequest.ExtendClimb -> SuperstructureStates.CLIMB_EXTEND
              is Request.SuperstructureRequest.RetractClimb ->
                SuperstructureStates.CLIMB_RETRACT
              else -> currentState
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
      SuperstructureStates.SCORE_PREP_CORAL -> {
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

        if (elevator.inputs.elevatorPosition > ElevatorConstants.ELEVATOR_HEIGHT_TO_CLEAR_ARM) {
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
        if (coralScoringLevel != CoralLevel.L1)
          arm.currentRequest =
            Request.ArmRequest.ClosedLoop(ArmTunableValues.Angles.idleCoralAngle.get())
        armRollers.currentRequest =
          ArmRollersRequest.OpenLoop(ArmRollersConstants.OUTTAKE_CORAL_VOLTAGE)

        if (currentRequest is Request.SuperstructureRequest.Idle || arm.isAtTargetedPosition) {
          nextState = SuperstructureStates.IDLE
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

        if (elevator.inputs.elevatorPosition >= ElevatorConstants.ELEVATOR_HEIGHT_TO_CLEAR_ARM) {
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
        if (currentRequest is Request.SuperstructureRequest.Idle ||
          Clock.fpgaTime - lastTransitionTime > 2.0.seconds
        )
          nextState = SuperstructureStates.IDLE
      }
    }

    if (nextState != currentState) lastTransitionTime = Clock.fpgaTime
  }

  companion object {
    enum class SuperstructureStates {
      UNINITIALIZED,
      HOME,
      IDLE,
      GROUND_INTAKE_CORAL,
      GROUND_INTAKE_CORAL_CLEANUP,
      INTAKE_CORAL_INTO_ARM,
      INTAKE_ALGAE,
      CLIMB_EXTEND,
      CLIMB_RETRACT,
      SCORE_PREP_CORAL,
      SCORE_CORAL,
      PREP_SCORE_ALGAE,
      SCORE_ALGAE
    }
  }
}
