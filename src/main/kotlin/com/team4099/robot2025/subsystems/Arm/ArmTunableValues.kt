package com.team4099.robot2025.subsystems.Arm

import com.team4099.lib.logging.LoggedTunableValue
import com.team4099.robot2025.config.constants.ArmConstants
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inDegrees
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.inVoltsPerDegree
import org.team4099.lib.units.derived.inVoltsPerDegreePerSecond
import org.team4099.lib.units.derived.inVoltsPerDegreeSeconds
import org.team4099.lib.units.derived.inVoltsPerRadianPerSecond
import org.team4099.lib.units.derived.inVoltsPerRadianPerSecondPerSecond
import org.team4099.lib.units.derived.perDegree
import org.team4099.lib.units.derived.perDegreePerSecond
import org.team4099.lib.units.derived.perDegreeSeconds
import org.team4099.lib.units.derived.perRadianPerSecond
import org.team4099.lib.units.derived.perRadianPerSecondPerSecond
import org.team4099.lib.units.derived.volts

object ArmTunableValues {

  val armkS = LoggedTunableValue("arm/kS", Pair({ it.inVolts }, { it.volts }))
  val armkV =
    LoggedTunableValue(
      "arm/kV", Pair({ it.inVoltsPerRadianPerSecond }, { it.volts.perRadianPerSecond })
    )

  val armkA =
    LoggedTunableValue(
      "arm/kA",
      Pair({ it.inVoltsPerRadianPerSecondPerSecond }, { it.volts.perRadianPerSecondPerSecond })
    )
  val armkG = LoggedTunableValue("arm/kG", Pair({ it.inVolts }, { it.volts }))

  val armkP = LoggedTunableValue("arm/kP", Pair({ it.inVoltsPerDegree }, { it.volts.perDegree }))
  val armkI =
    LoggedTunableValue(
      "arm/kI", Pair({ it.inVoltsPerDegreeSeconds }, { it.volts.perDegreeSeconds })
    )
  val armkD =
    LoggedTunableValue(
      "arm/kD", Pair({ it.inVoltsPerDegreePerSecond }, { it.volts.perDegreePerSecond })
    )

  object Angles {
    val idleAngle =
      LoggedTunableValue(
        "arm/idleAngle", ArmConstants.ANGLES.IDLE_ANGLE, Pair({ it.inDegrees }, { it.degrees })
      )

    val idleCoralAngle =
      LoggedTunableValue(
        "arm/idleCoralAngle",
        ArmConstants.ANGLES.IDLE_CORAL_ANGLE,
        Pair({ it.inDegrees }, { it.degrees })
      )
    val idleAlgaeAngle =
      LoggedTunableValue(
        "arm/idleAlgaeAngle",
        ArmConstants.ANGLES.IDLE_ALGAE_ANGLE,
        Pair({ it.inDegrees }, { it.degrees })
      )

    val hardstopIntakeAngle =
      LoggedTunableValue(
        "arm/hardstopIntakeAngle",
        ArmConstants.ANGLES.INTAKE_CORAL_ANGLE,
        Pair({ it.inDegrees }, { it.degrees })
      )

    val l1PrepAngle =
      LoggedTunableValue(
        "arm/l1PrepAngle",
        ArmConstants.ANGLES.L1_PREP_ANGLE,
        Pair({ it.inDegrees }, { it.degrees })
      )
    val l2PrepAngle =
      LoggedTunableValue(
        "arm/l2PrepAngle",
        ArmConstants.ANGLES.L2_PREP_ANGLE,
        Pair({ it.inDegrees }, { it.degrees })
      )
    val l3PrepAngle =
      LoggedTunableValue(
        "arm/l3PrepAngle",
        ArmConstants.ANGLES.L3_PREP_ANGLE,
        Pair({ it.inDegrees }, { it.degrees })
      )
    val l4PrepAngle =
      LoggedTunableValue(
        "arm/l4PrepAngle",
        ArmConstants.ANGLES.L4_PREP_ANGLE,
        Pair({ it.inDegrees }, { it.degrees })
      )

    val bargeAngle =
      LoggedTunableValue(
        "arm/bargeAngle",
        ArmConstants.ANGLES.BARGE_ANGLE,
        Pair({ it.inDegrees }, { it.degrees })
      )
    val processorAngle =
      LoggedTunableValue(
        "arm/processorAngle",
        ArmConstants.ANGLES.PROCESSOR_ANGLE,
        Pair({ it.inDegrees }, { it.degrees })
      )

    val algaeGroundIntakeAngle =
      LoggedTunableValue(
        "arm/algaeGroundIntakeAngle",
        ArmConstants.ANGLES.ALGAE_GROUND_INTAKE_ANGLE,
        Pair({ it.inDegrees }, { it.degrees })
      )
    val algaeLowIntakeAngle =
      LoggedTunableValue(
        "arm/algaeLowIntakeAngle",
        ArmConstants.ANGLES.ALGAE_LOW_INTAKE_ANGLE,
        Pair({ it.inDegrees }, { it.degrees })
      )
    val algaeHighIntakeAngle =
      LoggedTunableValue(
        "arm/algaeHighIntakeAngle",
        ArmConstants.ANGLES.ALGAE_HIGH_INTAKE_ANGLE,
        Pair({ it.inDegrees }, { it.degrees })
      )

    val scoreOffset =
      LoggedTunableValue(
        "arm/scoreOffset",
        ArmConstants.ANGLES.SCORE_ANGLE_OFFSET,
        Pair({ it.inDegrees }, { it.degrees })
      )

    val movingBetweenReefLevelsAngles =
      LoggedTunableValue(
        "arm/movingBetweenReefLevelsAngles",
        ArmConstants.ANGLES.MOVING_BETWEEN_REEF_LEVELS_ANGLE,
        Pair({ it.inDegrees }, { it.degrees })
      )
  }
}
