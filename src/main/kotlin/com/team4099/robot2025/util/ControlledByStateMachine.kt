package com.team4099.robot2025.util

import edu.wpi.first.util.sendable.Sendable
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.util.sendable.SendableRegistry

abstract class ControlledByStateMachine(name: String?) : Sendable {
  constructor() : this(null)

  init {
    val sentName = name ?: this.javaClass.simpleName
    SendableRegistry.addLW(this, sentName, sentName)
  }

  abstract fun loop()

  final override fun initSendable(builder: SendableBuilder) {
    builder.setSmartDashboardType("Subsystem");
  }
}