package com.team4099.robot2025.subsystems.dashboard

import edu.wpi.first.net.WebServer
import edu.wpi.first.networktables.BooleanArrayPublisher
import edu.wpi.first.networktables.BooleanArraySubscriber
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.networktables.PubSubOption
import edu.wpi.first.networktables.StringArrayPublisher
import edu.wpi.first.networktables.StringArraySubscriber
import edu.wpi.first.wpilibj.Filesystem
import java.nio.file.Paths

object ReefControlsIOServer : ReefControlsIO {

  private const val toRobotTable = "/ReefControls/ToRobot"
  private const val toDashboardTable = "/ReefControls/ToDashboard"
  private const val coralTopicName = "Coral"
  private const val algaeTopicName = "Algae"
  private const val prioritiesTopicName = "Priorities"

  private val coralStateIn: BooleanArraySubscriber
  private val algaeStateIn: BooleanArraySubscriber
  private val prioritiesIn: StringArraySubscriber

  private val coralStateOut: BooleanArrayPublisher
  private val algaeStateOut: BooleanArrayPublisher
  private val prioritiesOut: StringArrayPublisher

  init {
    // Create subscribers with default values
    val inputTable = NetworkTableInstance.getDefault().getTable(toRobotTable)
    coralStateIn =
      inputTable
        .getBooleanArrayTopic(coralTopicName)
        .subscribe(
          BooleanArray(36) { false },
          PubSubOption.keepDuplicates(true),
          PubSubOption.sendAll(true)
        )
    algaeStateIn =
      inputTable
        .getBooleanArrayTopic(algaeTopicName)
        .subscribe(
          BooleanArray(6) { false },
          PubSubOption.keepDuplicates(true),
          PubSubOption.sendAll(true)
        )
    prioritiesIn =
      inputTable
        .getStringArrayTopic(prioritiesTopicName)
        .subscribe(
          arrayOf("Fill L4", "Fill L3", "Fill L2"),
          PubSubOption.keepDuplicates(true),
          PubSubOption.sendAll(true)
        )

    // Create publishers
    val outputTable = NetworkTableInstance.getDefault().getTable(toDashboardTable)
    coralStateOut = outputTable.getBooleanArrayTopic(coralTopicName).publish()
    algaeStateOut = outputTable.getBooleanArrayTopic(algaeTopicName).publish()
    prioritiesOut = outputTable.getStringArrayTopic(prioritiesTopicName).publish()

    // Start web server
    try {
      WebServer.start(
        5801,
        Paths.get(Filesystem.getDeployDirectory().getAbsolutePath().toString(), "dashboard")
          .toString()
      )
      println("ReefControls: Web server started on port 5801")
    } catch (e: Exception) {
      println("ReefControls: Failed to start web server: ${e.message}")
    }
  }

  override fun updateInputs(inputs: ReefControlsIO.ReefControlsIOInputs) {
    try {
      // Use get() instead of readQueue() - safer
      inputs.coralState = coralStateIn.get()
      inputs.algaeState = algaeStateIn.get()
      inputs.priorities = prioritiesIn.get()
    } catch (e: Exception) {
      println("ReefControlsIO updateInputs error: ${e.message}")
      e.printStackTrace()
      inputs.coralState = BooleanArray(36) { false }
      inputs.algaeState = BooleanArray(6) { false }
      inputs.priorities = arrayOf("Fill L4", "Fill L3", "Fill L2")
    }
  }

  override fun setCoralState(value: BooleanArray) {
    try {
      coralStateOut.set(value)
    } catch (e: Exception) {
      println("ReefControlsIO setCoralState error: ${e.message}")
    }
  }

  override fun setAlgaeState(value: BooleanArray) {
    try {
      algaeStateOut.set(value)
    } catch (e: Exception) {
      println("ReefControlsIO setAlgaeState error: ${e.message}")
    }
  }

  override fun setPriorities(value: Array<String>) {
    try {
      prioritiesOut.set(value)
    } catch (e: Exception) {
      println("ReefControlsIO setPriorities error: ${e.message}")
    }
  }
}
