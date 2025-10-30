package com.team4099.robot2025.util

// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

abstract class VirtualSubsystem {
  init {
    subsystems.add(this)
  }

  abstract fun periodic()

  companion object {
    private val subsystems: MutableList<VirtualSubsystem> = ArrayList<VirtualSubsystem>()

    fun periodicAll() {
      for (subsystem in subsystems) {
        subsystem.periodic()
      }
    }
  }
}