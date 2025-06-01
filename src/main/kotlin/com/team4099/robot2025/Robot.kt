package com.team4099.robot2025

import com.team4099.lib.hal.Clock
import com.team4099.robot2025.auto.AutonomousSelector
import com.team4099.robot2025.auto.PathStore
import com.team4099.robot2025.config.constants.Constants
import com.team4099.robot2025.util.Alert
import com.team4099.robot2025.util.Alert.AlertType
import com.team4099.robot2025.util.CustomLogger
import com.team4099.robot2025.util.FMSData
import com.team4099.robot2025.util.NTSafePublisher
import edu.wpi.first.hal.AllianceStationID
import edu.wpi.first.networktables.GenericEntry
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.PowerDistribution
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.livewindow.LiveWindow
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.simulation.DriverStationSim
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj2.command.Commands.runOnce
import org.ejml.EjmlVersion.BUILD_DATE
import org.ejml.EjmlVersion.DIRTY
import org.ejml.EjmlVersion.GIT_BRANCH
import org.ejml.EjmlVersion.GIT_SHA
import org.ejml.EjmlVersion.MAVEN_NAME
import org.littletonrobotics.junction.LogFileUtil
import org.littletonrobotics.junction.LoggedRobot
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.inputs.LoggedPowerDistribution
import org.littletonrobotics.junction.wpilog.WPILOGReader
import org.littletonrobotics.junction.wpilog.WPILOGWriter
import org.team4099.lib.units.base.inMilliseconds
import java.nio.file.Files
import java.nio.file.Paths

object Robot : LoggedRobot() {
  val logFolderAlert =
    Alert("Log folder path does not exist. Data will NOT be logged.", AlertType.ERROR)
  val logReceiverQueueAlert =
    Alert("Logging queue exceeded capacity, data will NOT be logged.", AlertType.ERROR)
  val logOpenFileAlert = Alert("Failed to open log file. Data will NOT be logged", AlertType.ERROR)
  val logWriteAlert =
    Alert("Failed write to the log file. Data will NOT be logged", AlertType.ERROR)
  val logSimulationAlert = Alert("Running in simulation", AlertType.INFO)
  val logTuningModeEnabled =
    Alert("Tuning Mode Enabled. Expect loop times to be greater", AlertType.WARNING)
  lateinit var allianceSelected: GenericEntry
  lateinit var autonomousCommand: Command
  lateinit var autonomousLoadingCommand: Command
  /*
  val port0 = AnalogInput(0)
  val port1 = AnalogInput(1)
  val port2 = AnalogInput(2)
  val port3 = AnalogInput(3)

   */

  override fun robotInit() {}

  override fun autonomousInit() {}

  override fun disabledPeriodic() {}

  override fun disabledInit() {}

  override fun robotPeriodic() {
  }

  override fun teleopInit() {}

  override fun testInit() {}
}
