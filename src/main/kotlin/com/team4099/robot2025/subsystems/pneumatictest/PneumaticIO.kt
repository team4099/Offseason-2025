package com.team4099.robot2025.subsystems.pneumatictest

import com.team4099.robot2025.config.constants.PneumaticConstants

interface PneumaticIO {
  fun setPower(output: PneumaticConstants.PneumaticOutputs) {}
}