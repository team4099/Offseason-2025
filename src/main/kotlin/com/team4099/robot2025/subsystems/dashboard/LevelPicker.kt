package com.team4099.robot2025.subsystems.dashboard

import com.team4099.robot2025.commands.drivetrain.CoolerTargetTagCommand
import com.team4099.robot2025.config.constants.Constants
import com.team4099.robot2025.subsystems.vision.Vision
import com.team4099.robot2025.util.CustomLogger
import edu.wpi.first.wpilibj2.command.SubsystemBase

class LevelPicker(val io: ReefControlsIO, vision: Vision) : SubsystemBase() {

  val inputs = ReefControlsIO.ReefControlsIOInputs()
  var lastAlignedBranch: IntArray = CoolerTargetTagCommand.LastAlignedBranch
  var levelDecision: Constants.Universal.CoralLevel = Constants.Universal.CoralLevel.NONE

  override fun periodic() {
    io.updateInputs(inputs)

    // Add safety checks before logging
    if (inputs.coralState.isNotEmpty()) {
      CustomLogger.recordOutput("Dashboard/ReefState/Coral", inputs.coralState)
    }
    if (inputs.algaeState.isNotEmpty()) {
      CustomLogger.recordOutput("Dashboard/ReefState/Algae", inputs.algaeState)
    }
    if (inputs.priorities.isNotEmpty()) {
      CustomLogger.recordOutput("Dashboard/ReefState/Priorities", inputs.priorities)
    }

    CustomLogger.recordOutput("Dashboard/LevelDecision", levelDecision)
    CustomLogger.recordOutput("Dashboard/LastAlignedBranch", lastAlignedBranch)

    lastAlignedBranch = CoolerTargetTagCommand.LastAlignedBranch

    // Add comprehensive safety checks
    if (!lastAlignedBranch.contentEquals(intArrayOf(0, 0, 0)) &&
      inputs.coralState.isNotEmpty() &&
      inputs.priorities.isNotEmpty()) {
      try {
        levelDecision = branchToDecision(lastAlignedBranch)
      } catch (e: Exception) {
        CustomLogger.recordOutput("Dashboard/Error", "branchToDecision failed: ${e.message}")
      }
    }
  }

  fun branchToDecision(lastAlignedBranch: IntArray): Constants.Universal.CoralLevel {
    // Validate array sizes before accessing
    if (lastAlignedBranch.size < 3) {
      CustomLogger.recordOutput("Dashboard/Error", "lastAlignedBranch too small")
      return Constants.Universal.CoralLevel.L1
    }

    val maxIndex = lastAlignedBranch.maxOrNull() ?: 0
    if (inputs.coralState.isEmpty() || inputs.coralState.size <= maxIndex) {
      CustomLogger.recordOutput("Dashboard/Error", "coralState invalid size")
      return Constants.Universal.CoralLevel.L1
    }

    if (inputs.priorities.isEmpty()) {
      CustomLogger.recordOutput("Dashboard/Error", "priorities is empty")
      return Constants.Universal.CoralLevel.L1
    }

    for (priority in inputs.priorities) {
      CustomLogger.recordOutput("Dashboard/priority", priority)
      when (priority) {
        "Fill L4" -> {
          val index = lastAlignedBranch[0]
          if (index >= 0 && index < inputs.coralState.size && !inputs.coralState[index]) {
            return Constants.Universal.CoralLevel.L4
          }
        }
        "Fill L3" -> {
          val index = lastAlignedBranch[1]
          if (index >= 0 && index < inputs.coralState.size && !inputs.coralState[index]) {
            return Constants.Universal.CoralLevel.L3
          }
        }
        "Fill L2" -> {
          val index = lastAlignedBranch[2]
          if (index >= 0 && index < inputs.coralState.size && !inputs.coralState[index]) {
            return Constants.Universal.CoralLevel.L2
          }
        }
        else -> {
          return Constants.Universal.CoralLevel.L1
        }
      }
    }
    return Constants.Universal.CoralLevel.L4
  }
}