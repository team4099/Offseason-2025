package com.team4099.robot2025.subsystems.superstructure

import com.team4099.lib.hal.Clock
import com.team4099.robot2025.config.constants.ElevatorConstants
import com.team4099.robot2025.subsystems.Arm.Arm
import com.team4099.robot2025.subsystems.Arm.ArmTunableValues
import com.team4099.robot2025.subsystems.drivetrain.drive.Drivetrain
import com.team4099.robot2025.subsystems.elevator.Elevator
import com.team4099.robot2025.subsystems.elevator.ElevatorTunableValues
import com.team4099.robot2025.subsystems.limelight.LimelightVision
import com.team4099.robot2025.subsystems.vision.Vision
import com.team4099.robot2025.util.CustomLogger
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.team4099.lib.units.base.seconds
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
  private val armRollers: ArmRollers
) : SubsystemBase() {

  val theoreticalGamePiece: GamePiece = GamePiece.NONE
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
        is Request.SuperstructureRequest.ScorePrepCoral -> {
          coralScoringLevel = value.level
        }
        is Request.SuperstructureRequest.ScorePrepAlgae -> {
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
    CustomLogger.recordOutput("Superstructure/theoreticalGamePiece", theoreticalGamePiece)

    var nextState = currentState
    when (currentState) {
      // General States
      SuperstructureStates.UNINITIALIZED -> {
        nextState = SuperstructureStates.HOME
      }
      SuperstructureStates.SCORE_CORAL_PREP -> {
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
              else -> Request.ArmRequest.ClosedLoop(ArmTunableValues.Angles.idleAngle.get())
            }
        }

        when (currentRequest) {
          is Request.SuperstructureRequest.Score -> {
            if (elevator.isAtTargetedPosition && arm.isAtTargetedPosition) nextState = SuperstructureStates.SCORE_CORAL
          }
          is Request.SuperstructureRequest.Idle -> {
            nextState = SuperstructureStates.IDLE
          }
        }
      }
      SuperstructureStates.SCORE_CORAL -> {
        if (coralScoringLevel != CoralLevel.L1)
          arm.currentRequest =
            Request.ArmRequest.ClosedLoop(ArmTunableValues.Angles.idleAngle.get())
        armRollers.currentRequest = ArmRollersRequest.OpenLoop(ArmRollersConstants.OUTTAKE_CORAL_VOLTAGE)

        if (currentRequest is Request.SuperstructureRequest.Idle || arm.isAtTargetedPosition) {
          nextState = SuperstructureStates.IDLE
        }
      }
      SuperstructureStates.SCORE_ALGAE_PREP -> {
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

        arm.currentRequest =
          when (algaeScoringLevel) {
            AlgaeScoringLevel.PROCESSOR ->
              Request.ArmRequest.ClosedLoop(ArmTunableValues.Angles.processorAngle.get())
            AlgaeScoringLevel.BARGE ->
              Request.ArmRequest.ClosedLoop(ArmTunableValues.Angles.bargeAngle.get())
            else -> Request.ArmRequest.ClosedLoop(ArmTunableValues.Angles.idleAngle.get())
          }

        when (currentRequest) {
          is Request.SuperstructureRequest.Idle -> nextState = SuperstructureStates.IDLE
          is Request.SuperstructureRequest.Score -> {
            if (elevator.isAtTargetedPosition && arm.isAtTargetedPosition) nextState = SuperstructureStates.SCORE_ALGAE
          }
        }
      }
      SuperstructureStates.SCORE_ALGAE -> {
        armRollers.currentRequest = ArmRollersRequest.OpenLoop(ArmRollersConstants.OUTTAKE_ALGAE_VOLTAGE)
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
      INTAKE_CORAL_INTO_ARM,
      INTAKE_ALGAE,
      CLIMB_EXTEND,
      CLIMB_RETRACT,
      SCORE_CORAL_PREP,
      SCORE_CORAL,
      SCORE_ALGAE_PREP,
      SCORE_ALGAE
    }
  }
}
