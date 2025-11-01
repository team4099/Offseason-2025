package com.team4099.robot2025.config.constants

import org.team4099.lib.units.base.grams
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.base.pounds
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.meterSquared
import org.team4099.lib.units.kilo
import org.team4099.lib.units.milli
import org.team4099.lib.units.perSecond

object Constants {
  object Universal {
    val gravity = -9.8.meters.perSecond.perSecond
    val SIM_MODE = Tuning.SimType.SIM
    const val REAL_FIELD = false

    const val CTRE_CONFIG_TIMEOUT = 0
    const val EPSILON = 1E-9

    val SLOW_STATUS_FRAME_TIME = 255.milli.seconds
    const val CANIVORE_NAME = "FalconVore"
    val LOG_FOLDER = "/media/sda1"

    val LOOP_PERIOD_TIME = 20.milli.seconds
    val POWER_DISTRIBUTION_HUB_ID = 1

    const val SIMULATE_VISION = true

    enum class GamePiece {
      CORAL,
      ALGAE,
      NONE
    }

    enum class CoralLevel {
      NONE,
      L1,
      L2,
      L3,
      L4
    }

    enum class AlgaeIntakeLevel {
      NONE,
      GROUND,
      L2,
      L3
    }

    enum class AlgaeScoringLevel {
      NONE,
      BARGE,
      PROCESSOR
    }

    val HIGH_ALGAE_REEF_TAGS = intArrayOf(7, 9, 11, 18, 20, 22)

    val ROBOT_WEIGHT = 135.pounds
    val ROBOT_MOI = 6.76.kilo.grams.meterSquared
  }

  object AprilTagIds {
    const val BLUE_DOUBLE_SUBSTATION_ID = 4
  }

  object Tuning {
    const val TUNING_MODE = false
    const val DEBUGING_MODE = false

    enum class SimType {
      SIM,
      REPLAY
    }
  }

  object Joysticks {
    const val DRIVER_PORT = 0
    const val SHOTGUN_PORT = 1
    const val TECHNICIAN_PORT = 2

    const val THROTTLE_DEADBAND = 0.05
    const val TURN_DEADBAND = 0.05
  }

  object Climber {
    const val CLIMBER_MOTOR_ID = 31
  }

  // leader is right motor and follower is left
  object Elevator {
    const val LEADER_MOTOR_ID = 41 // right
    const val FOLLOWER_MOTOR_ID = 42 // left
  }

  object Arm {
    const val ARM_MOTOR_ID = 51
    const val CANCODER_ID = 9
  }
  object ArmRollers {
    const val ARM_ROLLERS_MOTOR_ID = 55
  }

  object Rollers {
    const val ROLLERS_MOTOR_ID = 61
  }

  object Intake {
    const val INTAKE_MOTOR_ID = 25

    object Rollers {
      const val ROLLERS_MOTOR_ID = 26
    }
  }

  object Indexer {
    const val INDEXER_MOTOR_ID = 58
  }

  object Candle {
    const val CANDLE_ID_1 = 2
    const val CANDLE_ID_2 = 3
  }

  object Alert {
    val TABS = arrayOf("Pre-match", "In-match")
  }

  object CanRange {
    const val CANRANGE_ID = 27
  }
}
