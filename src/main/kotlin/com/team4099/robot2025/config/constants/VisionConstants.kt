package com.team4099.robot2025.config.constants

import org.team4099.lib.geometry.Pose2d
import org.team4099.lib.geometry.Rotation3d
import org.team4099.lib.geometry.Transform3d
import org.team4099.lib.geometry.Translation3d
import org.team4099.lib.units.base.inches
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.derived.degrees
import java.util.function.Supplier

object VisionConstants {
  const val SIM_POSE_TOPIC_NAME = "Odometry/groundTruthPose"
  const val POSE_TOPIC_NAME = "Odometry/pose"

  const val NUM_OF_CAMERAS = 2

  val BLUE_REEF_TAGS = arrayOf(17, 18, 19, 20, 21, 22)
  val RED_REEF_TAGS = arrayOf(6, 7, 8, 9, 10, 11)

  val BLUE_REEF_TAG_THETA_ALIGNMENTS =
    mapOf(
      17 to 60.degrees,
      18 to 0.degrees,
      19 to 300.degrees,
      20 to 240.degrees,
      21 to 180.degrees,
      22 to 120.degrees
    )

  val OTF_PATHS = mapOf(
21 to listOf(
  Supplier{ Pose2d(6.33.meters, 4.223.meters, 123.89.degrees)},
  Supplier{ Pose2d(2.29.meters,6.201.meters, -175.206.degrees )}
),

    20 to listOf(
      Supplier{ Pose2d(5.382.meters, 5.382.meters, 136.757.degrees)},
      Supplier{ Pose2d(1.606.meters,6.512.meters, 179.377.degrees )}
    ),
    19 to listOf(
      Supplier{ Pose2d(3.872.meters, 5.505.meters, 155.914.degrees)},
      Supplier{ Pose2d(1.606.meters,6.512.meters, 157.332.degrees )}
    ),
    18 to listOf(
      Supplier{ Pose2d(2.889.meters, 4.103.meters, 116.913.degrees)},
      Supplier{ Pose2d(1.606.meters,6.512.meters, 117.615.degrees )}
    ),
    10 to listOf(
      Supplier{ Pose2d(6.33.meters, 4.223.meters, 123.89.degrees)},
      Supplier{ Pose2d(2.29.meters,6.201.meters, -175.206.degrees )}
    ),
    7 to listOf(
      Supplier{ Pose2d(2.889.meters, 4.103.meters, 116.913.degrees)},
      Supplier{ Pose2d(1.606.meters,6.512.meters, 117.615.degrees )}
    ),
  )

  val BLUE_REEF_TAG_Y_ALIGNMENTS =
    mapOf(
      17 to Pair(6.5.inches, -6.5.inches),
      18 to Pair(6.5.inches, -6.5.inches),
      19 to Pair(6.5.inches, -6.5.inches),
      20 to Pair(-6.5.inches, 6.5.inches),
      21 to Pair(-6.5.inches, 6.5.inches),
      22 to Pair(-6.5.inches, 6.5.inches),
    )

  val RED_REEF_TAG_THETA_ALIGNMENTS =
    mapOf(
      6 to 120.degrees,
      7 to 180.degrees,
      8 to 240.degrees,
      9 to 300.degrees,
      10 to 0.degrees,
      11 to 60.degrees
    )

  val RED_REEF_TAG_Y_ALIGNMENTS =
    mapOf(
      6 to Pair(6.5.inches, -6.5.inches),
      7 to Pair(6.5.inches, -6.5.inches),
      8 to Pair(6.5.inches, -6.5.inches),
      9 to Pair(-6.5.inches, 6.5.inches),
      10 to Pair(-6.5.inches, 6.5.inches),
      11 to Pair(-6.5.inches, 6.5.inches),
    )

  val BLUE_STATION_ALIGN_THETA = 54.degrees

  val RED_STATION_ALIGN_THETA = 126.degrees

  val AMBIGUITY_THESHOLD = 1.0
  val XY_STDDEV = 0.05
  val THETA_STDDEV = 10.0

  val CAMERA_TRANSFORMS =
    listOf(
      Transform3d(
        Translation3d(10.49.inches, 12.325.inches, 8.195.inches), // 18.69
        Rotation3d(0.0.degrees, -20.degrees, -30.degrees)
      ), // l
      Transform3d(
        Translation3d(10.49.inches, -12.325.inches, 8.195.inches), // 18.69
        Rotation3d(0.0.degrees, -20.degrees, 30.degrees)
      )
    )

  val CAMERA_NAMES = listOf("raven_1", "raven_2")

  object Limelight {
    val LIMELIGHT_NAME = "limelight-owl"
    val HORIZONTAL_FOV = 59.6.degrees
    val VERITCAL_FOV = 45.7.degrees
    val HIGH_TAPE_HEIGHT = 43.875.inches + 1.inches
    val MID_TAPE_HEIGHT = 23.905.inches + 1.inches
    val LL_TRANSFORM =
      Transform3d(
        Translation3d(-14.655.inches, 0.inches, 23.316.inches),
        Rotation3d(0.degrees, 143.degrees, 180.degrees)
      )
    const val RES_WIDTH = 320
    const val RES_HEIGHT = 240
  }
}
