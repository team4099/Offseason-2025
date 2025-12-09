package com.team4099.robot2025.config.constants

import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import org.team4099.lib.apriltag.AprilTag
import org.team4099.lib.geometry.Pose3d
import org.team4099.lib.units.base.meters

// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

/**
 * Contains various field dimensions and useful reference points. Dimensions are in meters, and sets
 * of corners start in the lower left moving clockwise. **All units in Meters** <br></br> <br></br>
 *
 * All translations and poses are stored with the origin at the rightmost point on the BLUE ALLIANCE
 * wall.<br></br> <br></br> Length refers to the *x* direction (as described by wpilib) <br></br>
 * Width refers to the *y* direction (as described by wpilib)
 */
object FieldConstants {
  val FIELD_LAYOUT: AprilTagFieldLayout =
    AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField)

  val FIELD_LENGTH = FIELD_LAYOUT.fieldLength.meters
  val FIELD_WIDTH = FIELD_LAYOUT.fieldWidth.meters

  val FIELD_ORIGIN = FIELD_LAYOUT.origin

  /** @see getTagPose(id: Int) */
  val TAGS: List<AprilTag> = FIELD_LAYOUT.tags.map { tag -> AprilTag(tag) }

  fun getTagPose(id: Int): Pose3d {
    return Pose3d(FIELD_LAYOUT.getTagPose(id).orElse(edu.wpi.first.math.geometry.Pose3d.kZero))
  }
}
