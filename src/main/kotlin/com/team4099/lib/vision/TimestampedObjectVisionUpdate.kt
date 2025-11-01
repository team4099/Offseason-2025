package com.team4099.lib.vision

import org.team4099.lib.geometry.Transform3d
import org.team4099.lib.units.base.Time

data class TimestampedObjectVisionUpdate(
  val timestamp: Time,
  val targetClassID: Int,
  val robotTObject: Transform3d
)
