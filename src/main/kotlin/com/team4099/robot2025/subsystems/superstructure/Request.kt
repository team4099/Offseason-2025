package com.team4099.robot2025.subsystems.superstructure

import com.team4099.robot2025.config.constants.Constants
import org.team4099.lib.units.base.Length
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.ElectricalPotential

sealed interface Request {
  sealed interface SuperstructureRequest : Request {
    class Idle() : SuperstructureRequest

    class Home() : SuperstructureRequest

    class ExtendClimb() : SuperstructureRequest

    class RetractClimb() : SuperstructureRequest

    class IntakeCoral() : SuperstructureRequest

    class Eject() : SuperstructureRequest

    class IntakeAlgae(val level: Constants.Universal.AlgaeIntakeLevel) : SuperstructureRequest

    class PrepScoreAlgae(val level: Constants.Universal.AlgaeScoringLevel) : SuperstructureRequest

    class PrepScoreCoral(val level: Constants.Universal.CoralLevel) : SuperstructureRequest

    class Score() : SuperstructureRequest
  }

  sealed interface VisionRequest : Request {
    class TargetReef() : VisionRequest
    class TargetTag(val tags: Array<Int>) : VisionRequest
  }

  sealed interface RollersRequest : Request {
    class OpenLoop(val voltage: ElectricalPotential) : RollersRequest
  }

  sealed interface ArmRequest : Request {
    class OpenLoop(val armVoltage: ElectricalPotential) : ArmRequest
    class ClosedLoop(val armPosition: Angle) : ArmRequest
    class Home() : ArmRequest
  }

  sealed interface ClimberRequest : Request {
    class OpenLoop(
      val climberVoltage: ElectricalPotential,
      val rollersVoltage: ElectricalPotential
    ) : ClimberRequest
    class ClosedLoop(val position: Angle) : ClimberRequest
    class Home() : ClimberRequest
  }

  sealed interface ElevatorRequest : Request {
    class ClosedLoop(val position: Length) : ElevatorRequest
    class OpenLoop(val voltage: ElectricalPotential) : ElevatorRequest
    class Home() : ElevatorRequest
  }

  sealed interface IntakeRequest : Request {
    class OpenLoop(val pivotVoltage: ElectricalPotential, val rollersVoltage: ElectricalPotential) :
      IntakeRequest

    class TargetingPosition(val pivotPosition: Angle, val rollersVoltage: ElectricalPotential) :
      IntakeRequest

    class ZeroPivot() : IntakeRequest
  }

  sealed interface IndexerRequest : Request {
    class Idle : IndexerRequest
    class Index(val voltage: ElectricalPotential) : IndexerRequest
    class Eject : IndexerRequest
  }
}
