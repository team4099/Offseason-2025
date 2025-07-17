package com.team4099.robot2025.subsystems.superstructure

import com.team4099.robot2025.subsystems.drivetrain.drive.Drivetrain
import com.team4099.robot2025.subsystems.limelight.LimelightVision
import com.team4099.robot2025.subsystems.vision.Vision
import edu.wpi.first.wpilibj2.command.SubsystemBase

class Superstructure(
  private val drivetrain: Drivetrain,
  private val vision: Vision,
  private val limelight: LimelightVision
) : SubsystemBase() {
  override fun periodic() {}
}
