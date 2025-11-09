package com.team4099.robot2025.subsystems.dashboard

import com.team4099.robot2025.commands.drivetrain.CoolerTargetTagCommand
import com.team4099.robot2025.config.constants.Constants
import com.team4099.robot2025.subsystems.vision.Vision
import com.team4099.robot2025.util.CustomLogger
import com.team4099.robot2025.util.VirtualSubsystem
import edu.wpi.first.wpilibj2.command.SubsystemBase

class LevelPicker(val io: ReefControlsIO, vision: Vision) : SubsystemBase() {

  val inputs = ReefControlsIO.ReefControlsIOInputs()
  var lastAlignedBranch: IntArray = CoolerTargetTagCommand.LastAlignedBranch
  var levelDecision: Constants.Universal.CoralLevel = Constants.Universal.CoralLevel.NONE

  override fun periodic() {
    io.updateInputs(inputs)
    CustomLogger.recordOutput("Dashboard/ReefState/Coral", inputs.coralState)
    CustomLogger.recordOutput("Dashboard/ReefState/Algae", inputs.algaeState)
    CustomLogger.recordOutput("Dashboard/ReefState/Priorities", inputs.priorities)

    CustomLogger.recordOutput("Dashboard/LevelDecision", levelDecision)
    CustomLogger.recordOutput("Dashboard/LastAlignedBranch", lastAlignedBranch)
     lastAlignedBranch = CoolerTargetTagCommand.LastAlignedBranch
    if (!lastAlignedBranch.contentEquals(intArrayOf(0, 0, 0))) {
      levelDecision = branchToDecision(lastAlignedBranch)
    }
  }

  fun branchToDecision(lastAlignedBranch: IntArray): Constants.Universal.CoralLevel {
    for (priority in inputs.priorities) {
      CustomLogger.recordOutput("Dashboard/priority", priority)
      when (priority) {
        "Fill L4" -> {
          if (!inputs.coralState[lastAlignedBranch[0]]) {
            return Constants.Universal.CoralLevel.L4
          }
        }
        "Fill L3" -> {
          if (!inputs.coralState[lastAlignedBranch[1]]) {
            return Constants.Universal.CoralLevel.L3
          }
        }
        "Fill L2" -> {
          if (!inputs.coralState[lastAlignedBranch[2]]) {
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
