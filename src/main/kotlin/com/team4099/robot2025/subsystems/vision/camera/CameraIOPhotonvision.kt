package com.team4099.robot2025.subsystems.vision.camera

import com.team4099.robot2025.config.constants.VisionConstants
import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3
import org.photonvision.PhotonCamera
import org.photonvision.PhotonPoseEstimator
import org.photonvision.PhotonPoseEstimator.PoseStrategy
import org.photonvision.simulation.PhotonCameraSim
import org.team4099.lib.geometry.Transform3d
import java.util.function.Supplier

class CameraIOPhotonvision(
  override val identifier: String,
  override val transform: Transform3d,
  override val poseMeasurementConsumer: (Pose2d?, Double, Matrix<N3?, N1?>) -> Unit,
  val drivetrainRotationSupplier: Supplier<Rotation2d>
) : CameraIO {

  override val photonEstimator: PhotonPoseEstimator =
    PhotonPoseEstimator(
      AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark),
      PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
      transform.transform3d
    )
  override val camera: PhotonCamera = PhotonCamera(identifier)
  override var cameraSim: PhotonCameraSim? = null
  override var curStdDevs: Matrix<N3?, N1?> = VisionConstants.singleTagStdDevs

  init {
    photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY)
  }
}
