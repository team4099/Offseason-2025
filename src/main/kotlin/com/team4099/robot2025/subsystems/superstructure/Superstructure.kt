package com.team4099.robot2025.subsystems.superstructure

import com.team4099.lib.hal.Clock
import com.team4099.robot2025.config.constants.Constants.Universal.CoralLevel as CoralLevel
import com.team4099.robot2025.config.constants.Constants.Universal.AlgaeIntakeLevel as AlgaeIntakeLevel
import com.team4099.robot2025.config.constants.Constants.Universal.AlgaeScoringLevel as AlgaeScoringLevel
import com.team4099.robot2025.config.constants.Constants.Universal.GamePiece as GamePiece
import com.team4099.robot2025.subsystems.drivetrain.drive.Drivetrain
import com.team4099.robot2025.subsystems.limelight.LimelightVision
import com.team4099.robot2025.subsystems.vision.Vision
import com.team4099.robot2025.util.CustomLogger
import edu.wpi.first.wpilibj2.command.SubsystemBase

class Superstructure(
  private val drivetrain: Drivetrain,
  private val vision: Vision,
  private val limelight: LimelightVision
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
    }
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
