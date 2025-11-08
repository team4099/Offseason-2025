package com.team4099.robot2025.commands.drivetrain

import com.team4099.lib.hal.Clock
import com.team4099.robot2025.config.constants.DrivetrainConstants
import com.team4099.robot2025.subsystems.drivetrain.Drive
import com.team4099.robot2025.subsystems.vision.Vision
import com.team4099.robot2025.util.CustomLogger
import com.team4099.robot2025.util.driver.DriverProfile
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.Command
import org.team4099.lib.controller.PIDController
import org.team4099.lib.geometry.Transform2d
import org.team4099.lib.geometry.Translation2d
import org.team4099.lib.kinematics.ChassisSpeeds
import org.team4099.lib.units.Velocity
import org.team4099.lib.units.base.Time
import org.team4099.lib.units.base.inSeconds
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.Radian
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.perSecond
import kotlin.math.PI

class AutofaceReefCommand(private val drivetrain: Drive,
                          val driver: DriverProfile,
                          val driveX: () -> Double,
                          val driveY: () -> Double,
                          val turn: () -> Double,
                          val slowMode: () -> Boolean,
                          private val vision: Vision,
  ): Command() {
  private var startTime: Time = 0.0.seconds
  private val thetaPID: PIDController<Radian, Velocity<Radian>>
  init {
    if(RobotBase.isSimulation()){
      thetaPID = PIDController(DrivetrainConstants.PID.SIM_AUTO_THETA_PID_KP,DrivetrainConstants.PID.SIM_AUTO_THETA_PID_KI,
        DrivetrainConstants.PID.SIM_AUTO_THETA_PID_KD)
    }else if(DriverStation.isAutonomous()){
      thetaPID = PIDController(DrivetrainConstants.PID.AUTO_REEF_PID_KP,DrivetrainConstants.PID.AUTO_REEF_PID_KI,
        DrivetrainConstants.PID.AUTO_REEF_PID_KD)
    }else {
      thetaPID =
        PIDController(
          DrivetrainConstants.PID.TELEOP_THETA_PID_KP,
          DrivetrainConstants.PID.TELEOP_THETA_PID_KI,
          DrivetrainConstants.PID.TELEOP_THETA_PID_KD
        )
    }
    thetaPID.enableContinuousInput(-PI.radians, PI.radians)
  }
    override fun initialize() {
      startTime = Clock.fpgaTime

      thetaPID.reset()
      thetaPID.enableContinuousInput(-PI.radians, PI.radians)
      CustomLogger.recordOutput("AutofaceReefCommand/lastInitalized", startTime.inSeconds)
    }
  override fun execute() {
    CustomLogger.recordOutput("ActiveCommands/AutofaceReefCommand",true)

    val lastUpdate = vision.lastTrigVisionUpdate
    val odomTTag = lastUpdate.robotTReefTag

    val exists = odomTTag != Transform2d(Translation2d(), 0.degrees)
    CustomLogger.recordOutput("AutofaceReefCommand/odomTTagExists", exists)
    if (!exists || Clock.realTimestamp - lastUpdate.timestamp > .5.seconds)
      end(interrupted = true)
    val setpointRotation = odomTTag.rotation
    var thetavel =
      thetaPID.calculate(drivetrain.rotation, setpointRotation) *
          if (RobotBase.isReal()) -1.0 else 1.0
    val speed = driver.driveSpeedClampedSupplier(driveX, driveY, slowMode)
    val rotation = driver.rotationSpeedClampedSupplier(turn, slowMode)

         drivetrain.runSpeeds(
        ChassisSpeeds.fromFieldRelativeSpeeds(
          speed.first, speed.second, thetavel, drivetrain.pose.rotation
        )
      )

  }

  override fun end(interrupted: Boolean) {
    if (!interrupted) vision.isAligned = true
    vision.isAutoAligning = false

    CustomLogger.recordOutput("AutofaceReefCommand/interrupted", interrupted)

    drivetrain.runSpeeds(ChassisSpeeds())
    CustomLogger.recordOutput("ActiveCommands/AutofaceReefCommand", false)
  }
}