package com.team4099.robot2025.config.constants

import edu.wpi.first.math.Matrix
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3
import org.team4099.lib.geometry.Rotation3d
import org.team4099.lib.geometry.Transform3d
import org.team4099.lib.geometry.Translation3d
import org.team4099.lib.units.base.inches
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.derived.degrees

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
      6 to Pair(-6.5.inches, 6.5.inches),
      7 to Pair(-6.5.inches, 6.5.inches),
      8 to Pair(-6.5.inches, 6.5.inches),
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
        Translation3d(-10.398.inches, -11.593.inches, 8.36.inches),
        Rotation3d(0.0.degrees, -20.degrees, 150.degrees)
      ), // raven_1
      Transform3d(
        Translation3d(-10.398.inches, 11.593.inches, 8.36.inches),
        Rotation3d(0.0.degrees, -20.degrees, -150.degrees)
      ), // raven_2
    )

  val CAMERA_NAMES = listOf("raven_1", "raven_2")

  // x, y, θ
  // TODO tune
  val singleTagStdDevs: Matrix<N3?, N1?> = VecBuilder.fill(4.0, 4.0, 8.0)
  val multiTagStdDevs: Matrix<N3?, N1?> = VecBuilder.fill(0.5, 0.5, 1.0)

  val oldStdDevs: Matrix<N3?, N1?> = VecBuilder.fill(XY_STDDEV, XY_STDDEV, THETA_STDDEV)

  val FIELD_POSE_RESET_DISTANCE_THRESHOLD = 1.0.meters

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
