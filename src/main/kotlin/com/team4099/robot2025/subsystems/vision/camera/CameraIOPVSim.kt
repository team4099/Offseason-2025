package com.team4099.robot2025.subsystems.vision.camera

import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.math.geometry.Rotation2d
import org.photonvision.PhotonCamera
import org.photonvision.simulation.PhotonCameraSim
import org.photonvision.simulation.SimCameraProperties
import org.photonvision.simulation.VisionSystemSim
import org.team4099.lib.geometry.Transform3d
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inRotation2ds

class CameraIOPVSim(
  override val identifier: String,
  override val transform: Transform3d
) : CameraIO {
  private val cameraProperties: SimCameraProperties = SimCameraProperties()

  private val camera = PhotonCamera(identifier)

  override var cameraSim: PhotonCameraSim? = null

  init {

    cameraProperties.setCalibration(1280, 720, 100.0.degrees.inRotation2ds)
    cameraProperties.fps = 45.0

    cameraSim = PhotonCameraSim(camera, cameraProperties)
  }
}