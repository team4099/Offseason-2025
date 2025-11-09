package com.team4099.robot2025.subsystems.vision

import com.team4099.lib.hal.Clock
import com.team4099.lib.logging.TunableNumber
import com.team4099.lib.vision.TimestampedObjectVisionUpdate
import com.team4099.lib.vision.TimestampedTrigVisionUpdate
import com.team4099.lib.vision.TimestampedVisionUpdate
import com.team4099.robot2025.config.constants.Constants
import com.team4099.robot2025.config.constants.FieldConstants
import com.team4099.robot2025.config.constants.VisionConstants
import com.team4099.robot2025.subsystems.superstructure.Request
import com.team4099.robot2025.subsystems.vision.camera.CameraIO
import com.team4099.robot2025.util.CustomLogger
import com.team4099.robot2025.util.FMSData
import com.team4099.robot2025.util.toPose3d
import com.team4099.robot2025.util.toTransform3d
import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.ironmaple.simulation.SimulatedArena
import org.littletonrobotics.junction.Logger
import org.photonvision.PhotonUtils
import org.photonvision.simulation.VisionSystemSim
import org.team4099.lib.geometry.Pose2d
import org.team4099.lib.geometry.Pose3d
import org.team4099.lib.geometry.Rotation3d
import org.team4099.lib.geometry.Transform2d
import org.team4099.lib.geometry.Transform3d
import org.team4099.lib.geometry.Transform3dWPILIB
import org.team4099.lib.geometry.Translation2d
import org.team4099.lib.geometry.Translation3d
import org.team4099.lib.units.base.inInches
import org.team4099.lib.units.base.inMeters
import org.team4099.lib.units.base.inMilliseconds
import org.team4099.lib.units.base.inSeconds
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.cos
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inRadians
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.derived.sin
import java.util.function.Supplier

class Vision(vararg cameras: CameraIO, val poseSupplier: Supplier<Pose2d>) : SubsystemBase() {
  val io: List<CameraIO> = cameras.toList()
  val inputs = List(io.size) { CameraIO.CameraInputs() }

  var tagIDFilter = arrayOf<Int>()

  var currentState = VisionState.UNINITIALIZED
  var currentRequest: Request.VisionRequest = Request.VisionRequest.TargetReef()
    set(value) {
      when (value) {
        is Request.VisionRequest.TargetTag -> {
          tagIDFilter = value.tags
        }
        else -> {}
      }
      field = value
    }

  var isAutoAligning = false
  var isAligned = false

  private val xyStdDev = TunableNumber("Vision/xystdev", VisionConstants.XY_STDDEV)

  private val thetaStdDev = TunableNumber("Vision/thetaStdDev", VisionConstants.THETA_STDDEV)

  private var cameraPreference = 0 // 0 for left 1 for right

  private var closestReefTagAcrossCams: Map.Entry<Int, Pair<Int, Transform3d>?>? = null

  var lastTrigVisionUpdate =
    TimestampedTrigVisionUpdate(Clock.fpgaTime, -1, Transform2d(Translation2d(), 0.degrees))

  var objectsDetected: MutableList<MutableList<Translation2d>> =
    MutableList(VisionConstants.OBJECT_CLASS.values().size) { mutableListOf() }

  var lastObjectVisionUpdate: MutableList<TimestampedObjectVisionUpdate> =
    VisionConstants.OBJECT_CLASS
      .values()
      .map { TimestampedObjectVisionUpdate(Clock.fpgaTime, it, Translation2d()) }
      .toMutableList()

  private val fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField)

  private var lastSeenTagId: Int? = null
  private var pulseEndTime = 0.0.seconds
  var autoAlignReadyRumble = false
    private set

  private var visionSim: VisionSystemSim? = null

  init {
    if (RobotBase.isSimulation() && Constants.Universal.SIMULATE_VISION) {
      visionSim = VisionSystemSim("main")
      visionSim!!.addAprilTags(AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField))

      cameras.forEach { camera ->
        visionSim!!.addCamera(camera.cameraSim, camera.transform.transform3d)
      }
    }
  }

  override fun periodic() {
    val startTime = Clock.fpgaTime
    visionSim?.update(poseSupplier.get().pose2d)

    Logger.recordOutput(
      "Vision/cameraTransform1",
      edu.wpi.first.math.geometry.Pose3d()
        .transformBy(VisionConstants.CAMERA_TRANSFORMS[0].transform3d)
    )
    Logger.recordOutput(
      "Vision/cameraTransform2",
      edu.wpi.first.math.geometry.Pose3d()
        .transformBy(VisionConstants.CAMERA_TRANSFORMS[1].transform3d)
    )

    Logger.recordOutput("Vision/currentTrigUpdateID", lastTrigVisionUpdate.targetTagID)

    currentState = fromRequestToState(currentRequest)

    if (currentState == VisionState.TARGETING_REEF && DriverStation.getAlliance().isPresent) {
      tagIDFilter =
        if (FMSData.isBlue) VisionConstants.BLUE_REEF_TAGS else VisionConstants.RED_REEF_TAGS
    }

    for (instance in io.indices) {
      io[instance].updateInputs(inputs[instance])
      Logger.processInputs("Vision/${VisionConstants.CAMERA_NAMES[instance]}", inputs[instance])
    }

    val visionUpdates = mutableListOf<TimestampedVisionUpdate>()

    val closestReefTags = mutableMapOf<Int, Pair<Int, Transform3d>?>()
    for (i in io.indices) {
      closestReefTags[i] = null
    }

    for (instance in io.indices) {

      when (io[instance].pipeline) {
        CameraIO.DetectionPipeline.APRIL_TAG -> {
          var reefTags = mutableListOf<Pair<Int, Transform3d>>()
          var closestReefTag: Pair<Int, Transform3d>? = null

          var tagTargets = inputs[instance].cameraTargets.filter { it.fiducialId != -1 }

          val cornerData = mutableListOf<Double>()

          for (tag in tagTargets) {
            if (tag.poseAmbiguity < VisionConstants.AMBIGUITY_THESHOLD) {
              if (DriverStation.getAlliance().isPresent) {
                if ((tag.fiducialId in VisionConstants.BLUE_REEF_TAGS && FMSData.isBlue) ||
                  (tag.fiducialId in VisionConstants.RED_REEF_TAGS && !FMSData.isBlue)
                ) {

                  val aprilTagAlignmentAngle =
                    fieldLayout.getTagPose(tag.fiducialId).get().rotation.z.radians
                  //                if (FMSData.isBlue) {
                  //                  VisionConstants.BLUE_REEF_TAG_THETA_ALIGNMENTS[tag.fiducialId]
                  //                } else {
                  //                  VisionConstants.RED_REEF_TAG_THETA_ALIGNMENTS[tag.fiducialId]
                  //                }

                  val fieldTTag =
                    FieldConstants.AprilTagLayoutType.OFFICIAL
                      .layout
                      .getTagPose(tag.fiducialId)
                      .toTransform3d()

                  val cameraDistanceToTarget3D = tag.bestCameraToTarget.translation.norm.meters
                  val cameraDistanceToTarget2D = cameraDistanceToTarget3D * (tag.pitch.degrees).cos

                  var cameraTTagTranslation2d =
                    Translation2d(
                      PhotonUtils.estimateCameraToTargetTranslation(
                        cameraDistanceToTarget2D.inMeters,
                        Rotation2d(-tag.yaw.degrees.inRadians)
                      )
                    )

                  var cameraTTagTranslation3d =
                    Translation3d(
                      cameraTTagTranslation2d.x,
                      cameraTTagTranslation2d.y,
                      cameraDistanceToTarget3D * tag.pitch.degrees.sin
                    )

                  var cameraTTagRotation3d = Rotation3d(tag.bestCameraToTarget.rotation)

                  var robotTTag =
                    Transform3d(
                      Pose3d()
                        .transformBy(VisionConstants.CAMERA_TRANSFORMS[instance])
                        .transformBy(Transform3d(cameraTTagTranslation3d, Rotation3d()))
                        .translation,
                      Rotation3d(0.0.degrees, 0.0.degrees, aprilTagAlignmentAngle ?: 0.degrees)
                    )

                  var fieldTRobot = Pose3d().transformBy(fieldTTag).transformBy(robotTTag.inverse())

                  visionUpdates.add(
                    TimestampedVisionUpdate(
                      inputs[instance].timestamp,
                      fieldTRobot.toPose2d(),
                      VecBuilder.fill(xyStdDev.get(), xyStdDev.get(), thetaStdDev.get()),
                      true
                    )
                  )

                  val distanceToTarget = robotTTag.translation.norm

                  Logger.recordOutput(
                    "Vision/${VisionConstants.CAMERA_NAMES[instance]}/${tag.fiducialId}/cameraDistanceToTarget3D",
                    cameraDistanceToTarget3D.inInches
                  )

                  Logger.recordOutput(
                    "Vision/${VisionConstants.CAMERA_NAMES[instance]}/${tag.fiducialId}/cameraDistanceToTarget2D",
                    cameraDistanceToTarget2D.inInches
                  )

                  Logger.recordOutput(
                    "Vision/${VisionConstants.CAMERA_NAMES[instance]}/${tag.fiducialId}/robotDistanceToTarget",
                    distanceToTarget.inMeters
                  )

                  Logger.recordOutput(
                    "Vision/${VisionConstants.CAMERA_NAMES[instance]}/${tag.fiducialId}/fieldTTag",
                    fieldTTag.transform3d
                  )

                  Logger.recordOutput(
                    "Vision/${VisionConstants.CAMERA_NAMES[instance]}/${tag.fiducialId}/cameraTTag",
                    cameraTTagTranslation3d.translation3d
                  )

                  Logger.recordOutput(
                    "Vision/${VisionConstants.CAMERA_NAMES[instance]}/${tag.fiducialId}/robotTTag",
                    robotTTag.transform3d
                  )

                  Logger.recordOutput(
                    "Vision/${VisionConstants.CAMERA_NAMES[instance]}/${tag.fiducialId}/fieldTRobot",
                    fieldTRobot.pose3d
                  )

                  for (corner in tag.detectedCorners) {
                    cornerData.add(corner.x)
                    cornerData.add(corner.y)
                  }

                  if (tag.fiducialId in tagIDFilter) {
                    reefTags.add(Pair(tag.fiducialId, robotTTag))
                  }
                }
              }

              closestReefTag = reefTags.minByOrNull { it.second.translation.norm }

              closestReefTags[instance] = closestReefTag

              Logger.recordOutput(
                "Vision/${VisionConstants.CAMERA_NAMES[instance]}/cornerDetections}",
                cornerData.toDoubleArray()
              )

              Logger.recordOutput(
                "Vision/${VisionConstants.CAMERA_NAMES[instance]}/closestReefTagID}",
                closestReefTag?.first ?: -1
              )

              Logger.recordOutput(
                "Vision/${VisionConstants.CAMERA_NAMES[instance]}/closestReefTagPose}",
                closestReefTag?.second?.transform3d ?: Transform3dWPILIB()
              )
            }

            Logger.recordOutput(
              "Vision/viewingSameTag", closestReefTags[0]?.first == closestReefTags[1]?.first
            )

            closestReefTagAcrossCams =
              if (closestReefTags[0]?.first != closestReefTags[1]?.first) {
                closestReefTags.minByOrNull {
                  it.value?.second?.translation?.norm ?: 1000000.meters
                }
              } else {
                mapOf(cameraPreference to closestReefTags[cameraPreference]).minByOrNull {
                  it.value?.second?.translation?.norm ?: 1000000.meters
                }
              }

            Logger.recordOutput(
              "Vision/ClosestReefTagAcrossAllCams/CameraID",
              if (closestReefTagAcrossCams != null)
                VisionConstants.CAMERA_NAMES[closestReefTagAcrossCams?.key ?: -1]
              else "None"
            )

            Logger.recordOutput(
              "Vision/ClosestReefTagAcrossAllCams/TagID",
              closestReefTagAcrossCams?.value?.first ?: -1
            )

            Logger.recordOutput(
              "Vision/ClosestReefTagAcrossAllCams/ReefTagPose",
              closestReefTagAcrossCams?.value?.second?.transform3d ?: Transform3dWPILIB()
            )

            if (closestReefTagAcrossCams?.key != null && closestReefTagAcrossCams?.value != null) {

              lastTrigVisionUpdate =
                TimestampedTrigVisionUpdate(
                  inputs[closestReefTagAcrossCams?.key ?: 0].timestamp,
                  closestReefTagAcrossCams?.value?.first ?: -1,
                  Transform2d(
                    Translation2d(
                      closestReefTagAcrossCams?.value?.second?.translation?.x ?: 0.meters,
                      closestReefTagAcrossCams?.value?.second?.translation?.y ?: 0.meters
                    ),
                    closestReefTagAcrossCams?.value?.second?.rotation?.z ?: 0.degrees
                  )
                )
            }
          }
        }
        CameraIO.DetectionPipeline.OBJECT_DETECTION -> {
          val objTargets =
            inputs[instance].cameraTargets.filter {
              it.objDetectId != -1 && it.objDetectConf >= VisionConstants.CONFIDENCE_THRESHOLD
            }

          if (RobotBase.isReal()) {
            objectsDetected =
              MutableList(VisionConstants.OBJECT_CLASS.values().size) { mutableListOf() }

            for (idx in objTargets.indices) {
              // object pose detection credit to 5990 TRIGON Robot Template on Github, available at
              // https://github.com/Programming-TRIGON/RobotTemplate/blob/2d24e98f5e7f5b22657669d6d2a23f5c06f8231d/
              // src/main/java/frc/trigon/robot/misc/objectdetectioncamera/ObjectDetectionCamera.java#L62
              val rotation =
                Rotation3d(
                  0.radians, -objTargets[idx].pitch.degrees, -objTargets[idx].yaw.degrees
                )

              val cameraRotationToObject =
                VisionConstants.CAMERA_TRANSFORMS[instance].toPose3d()
                  .plus(Transform3d(Translation3d(), rotation))
              val xTransform =
                VisionConstants.CAMERA_TRANSFORMS[instance].z /
                  cameraRotationToObject.rotation.y.sin

              val robotTObject =
                cameraRotationToObject
                  .transformBy(
                    Transform3d(Translation3d(xTransform, 0.meters, 0.meters), Rotation3d())
                  )
                  .toPose2d()
                  .translation

              when (objTargets[idx].detectedObjectClassID) {
                VisionConstants.OBJECT_CLASS.CORAL.id -> {
                  objectsDetected[VisionConstants.OBJECT_CLASS.CORAL.id].add(robotTObject)
                }
                VisionConstants.OBJECT_CLASS.ALGAE.id -> {
                  objectsDetected[VisionConstants.OBJECT_CLASS.ALGAE.id].add(robotTObject)
                }
              }
            }
          } else {
            objectsDetected =
              MutableList(VisionConstants.OBJECT_CLASS.values().size) { mutableListOf() }

            for (objIdx in VisionConstants.OBJECT_CLASS.values().indices) {
              objectsDetected[objIdx].addAll(
                SimulatedArena.getInstance()
                  .getGamePiecesByType(
                    VisionConstants.OBJECT_CLASS.values()[objIdx].mapleSimType!!
                  )
                  .map {
                    Transform2d(poseSupplier.get(), Pose2d(it.pose3d.toPose2d())).translation
                  }
              )
            }

            objectsDetected.forEach {
              it.addAll(
                SimulatedArena.getInstance().getGamePiecesByType("CoralAlgaeStack").map { pose ->
                  Transform2d(poseSupplier.get(), Pose2d(pose.pose3d.toPose2d())).translation
                }
              )
            }
          }

          for (objects in VisionConstants.OBJECT_CLASS.values()) {
            val closestObject = objectsDetected[objects.id].minByOrNull { it.translation2d.norm }

            if (closestObject != null) {
              lastObjectVisionUpdate[objects.id] =
                TimestampedObjectVisionUpdate(inputs[instance].timestamp, objects, closestObject)
            }

            CustomLogger.recordOutput(
              "Vision/${VisionConstants.CAMERA_NAMES[instance]}/${objects.name}/objectsDetectedPoses",
              *(
                objectsDetected[objects.id]
                  .map { poseSupplier.get().plus(Transform2d(it, 0.radians)).pose2d }
                  .toTypedArray()
                )
            )

            CustomLogger.recordOutput(
              "Vision/Last${objects.name}VisionUpdate/timestampSeconds",
              lastObjectVisionUpdate[objects.id].timestamp.inSeconds
            )

            CustomLogger.recordOutput(
              "Vision/Last${objects.name}VisionUpdate/robotTObject",
              lastObjectVisionUpdate[objects.id].robotTObject.translation2d
            )

            CustomLogger.recordOutput(
              "Vision/Last${objects.name}VisionUpdate/closestObjectPose",
              poseSupplier
                .get()
                .plus(Transform2d(lastObjectVisionUpdate[objects.id].robotTObject, 0.radians))
                .pose2d
            )
          }
        }
      }
    }

    val now = Clock.fpgaTime

    val tagId0 = closestReefTags[0]?.first
    val tagId1 = closestReefTags[1]?.first

    val bothSeeingSameTag = tagId0 != null && tagId1 != null && tagId0 == tagId1

    val currentTagId = if (bothSeeingSameTag) tagId0 else null
    val distanceToTag = closestReefTagAcrossCams?.value?.second?.translation?.norm ?: 1000000.meters

    CustomLogger.recordOutput(
      "Vision/rumble",
      currentTagId != null &&
        currentTagId in tagIDFilter &&
        distanceToTag <= VisionConstants.CONTROLLER_RUMBLE_DIST
    )

    if (currentTagId != null &&
      currentTagId in tagIDFilter &&
      distanceToTag <= VisionConstants.CONTROLLER_RUMBLE_DIST
    ) {
      if (lastSeenTagId == null || currentTagId != lastSeenTagId) {
        pulseEndTime = now + 0.25.seconds
        autoAlignReadyRumble = true
      }
      lastSeenTagId = currentTagId
    } else if (currentTagId == null) {
      lastSeenTagId = null
    }

    if (now > pulseEndTime) {
      autoAlignReadyRumble = false
    }

    Logger.recordOutput(
      "LoggedRobot/Subsystems/VisionLoopTimeMS", (Clock.realTimestamp - startTime).inMilliseconds
    )
  }

  companion object {
    enum class VisionState {
      UNINITIALIZED,
      TARGETING_REEF,
      TARGETING_TAG
    }

    fun fromRequestToState(request: Request.VisionRequest): VisionState {
      return when (request) {
        is Request.VisionRequest.TargetReef -> VisionState.TARGETING_REEF
        is Request.VisionRequest.TargetTag -> VisionState.TARGETING_TAG
      }
    }
  }
}
