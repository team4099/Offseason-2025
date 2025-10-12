package com.team4099.robot2025.subsystems.vision.camera

import com.team4099.robot2025.config.constants.VisionConstants
import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3
import org.littletonrobotics.junction.Logger
import org.photonvision.EstimatedRobotPose
import org.photonvision.PhotonCamera
import org.photonvision.PhotonPoseEstimator
import org.photonvision.PhotonPoseEstimator.PoseStrategy
import org.photonvision.targeting.PhotonTrackedTarget
import org.team4099.lib.geometry.Pose3d
import org.team4099.lib.geometry.Transform3d
import org.team4099.lib.units.base.Time
import org.team4099.lib.units.base.inMeters
import org.team4099.lib.units.base.inSeconds
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.micro
import java.util.Optional
import java.util.function.Supplier
import org.team4099.lib.units.derived.Angle

class CameraIOPhotonvision(
  private val identifier: String,
  transform: Transform3d,
  val poseMeasurementConsumer: (Pose2d?, Double, Matrix<N3?, N1?>) -> Unit,
  val drivetrainRotationSupplier: Supplier<Angle>
) : CameraIO {

  private val photonEstimator: PhotonPoseEstimator =
    PhotonPoseEstimator(
      AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark),
      PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
      transform.transform3d
    )
  private val camera: PhotonCamera = PhotonCamera(identifier)
  private var lastEstTimestamp: Time = 0.0.seconds

  private var curStdDevs: Matrix<N3?, N1?> = VisionConstants.singleTagStdDevs

  init {
    photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY)
  }

  override fun updateInputs(inputs: CameraIO.CameraInputs) {
    if (camera.isConnected) {
      if (camera.cameraMatrix.isPresent) {
        inputs.cameraMatrix = camera.cameraMatrix.get()
      }

      if (camera.distCoeffs.isPresent) {
        inputs.distCoeff = camera.distCoeffs.get()
      }
    }

    val unreadResults = camera.allUnreadResults

    if (unreadResults.isEmpty()) return

    val mostRecentPipelineResult = unreadResults[unreadResults.size - 1]

    inputs.timestamp = mostRecentPipelineResult.timestampSeconds.seconds
    Logger.recordOutput("Vision/$identifier/timestampIG", mostRecentPipelineResult.timestampSeconds)

    inputs.cameraTargets = mostRecentPipelineResult.targets

    if (mostRecentPipelineResult.hasTargets()) {
      if ((inputs.timestamp - lastEstTimestamp).absoluteValue > 10.micro.seconds) {
        inputs.fps = 1 / (inputs.timestamp - lastEstTimestamp).inSeconds
        lastEstTimestamp = inputs.timestamp
      }

      val visionEst: Optional<EstimatedRobotPose> = photonEstimator.update(mostRecentPipelineResult)

      if (visionEst.isPresent) {
        inputs.usedTargets = visionEst.get().targetsUsed.map { it.fiducialId }

        val poseEst = visionEst.get().estimatedPose
        inputs.frame = Pose3d(poseEst)

        if (mostRecentPipelineResult.bestTarget.bestCameraToTarget.translation.norm <
          VisionConstants.FIELD_POSE_RESET_DISTANCE_THRESHOLD.inMeters
        ) {
          updateEstimationStdDevs(visionEst, mostRecentPipelineResult.getTargets())

          val poseEst2d = poseEst.toPose2d()

          poseMeasurementConsumer(poseEst2d, visionEst.get().timestampSeconds, curStdDevs)
        }
      }
    }
  }

  private fun updateEstimationStdDevs(
    estimatedPose: Optional<EstimatedRobotPose>?,
    targets: MutableList<PhotonTrackedTarget>
  ) {
    if (estimatedPose == null || estimatedPose.isEmpty) {
      curStdDevs = VisionConstants.singleTagStdDevs
      return
    }
    var estStdDevs = VisionConstants.singleTagStdDevs
    var numTags = 0
    var avgDist = 0.0

    // Precalculation - see how many tags we found, and calculate an average-distance metric
    for (tgt in targets) {
      val tagPose = photonEstimator.fieldTags.getTagPose(tgt.getFiducialId())
      if (tagPose.isEmpty) continue
      numTags++
      avgDist +=
        tagPose
          .get()
          .toPose2d()
          .translation
          .getDistance(estimatedPose.get().estimatedPose.toPose2d().translation)
    }

    if (numTags == 0) {
      curStdDevs = VisionConstants.singleTagStdDevs
    } else {
      // One or more tags visible, run the full heuristic.
      avgDist /= numTags.toDouble()
      // Decrease std devs if multiple targets are visible
      if (numTags > 1) estStdDevs = VisionConstants.multiTagStdDevs
      // Increase std devs based on (average) distance
      if (numTags == 1 && avgDist > 4)
        estStdDevs =
          VecBuilder.fill(
            Double.Companion.MAX_VALUE,
            Double.Companion.MAX_VALUE,
            Double.Companion.MAX_VALUE
          )
      else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30))
      curStdDevs = estStdDevs
    }
  }
}
