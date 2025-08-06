package com.team4099.robot2025.subsystems.superstructure

import com.team4099.robot2025.config.constants.Constants
import edu.wpi.first.math.kinematics.ChassisSpeeds
import org.team4099.lib.units.AngularVelocity
import org.team4099.lib.units.LinearVelocity
import org.team4099.lib.units.base.Length
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.ElectricalPotential

sealed interface Request {

  //    |    |              _.-7
  //           |\.-.|             ( ,(_
  //           | a a|              \\  \,
  //           ) ["||          _.--' \  \\
  //        .-'  '-''-..____.-'    ___)  )\
  //       F   _/-``-.__;-.-.--`--' . .' \_L_
  //      |   l  {~~} ,_\  '.'.      ` __.' )\
  //      (    -.;___,;  | '- _       :__.'( /
  //      | -.__ _/_.'.-'      '-._ .'      \\
  //      |     .'   |  -- _                 '\,
  //      |  \ /--,--{ .    '---.__.       .'  .'
  //      J  ;/ __;__]. '.-.            .-' )_/
  //      J  (-.     '\'. '. '-._.-.-'--._ /
  //      |  |  '. .' | \'. '.    ._       \
  //      |   \   T   |  \  '. '._  '-._    '.
  //      F   J   |   |  '.    .  '._   '-,_.--`
  //      F   \   \   F .  \    '.   '.  /
  //     J     \  |  J   \  '.   '.    '/
  //     J      '.L__|    .   \    '    |
  //     |   .    \  |     \   '.   '. /
  //     |    '    '.|      |    ,-.  (
  //     F   | ' ___  ',._   .  /   '. \
  //     F   (.'`|| (-._\ '.  \-      '-\
  //     \ .-'  ( L `._ '\ '._ (
  // snd  /'  |  /  '-._\      ''\

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

  sealed interface DrivetrainRequest : Request {
    class OpenLoop(
      val angularVelocity: AngularVelocity,
      val driveVector: Pair<LinearVelocity, LinearVelocity>,
      val fieldOriented: Boolean = true
    ) : DrivetrainRequest

    class ClosedLoop(
      var chassisSpeeds: ChassisSpeeds,
      val chassisAccels: ChassisSpeeds =
        edu.wpi.first.math.kinematics.ChassisSpeeds(0.0, 0.0, 0.0)
    ) : DrivetrainRequest

    class ZeroSensors(val isInAutonomous: Boolean = false) : DrivetrainRequest
    class Idle : DrivetrainRequest

    class LockWheels : DrivetrainRequest
    class Characterize(val voltage: ElectricalPotential) : DrivetrainRequest
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
    class Index : IndexerRequest
  }
}
