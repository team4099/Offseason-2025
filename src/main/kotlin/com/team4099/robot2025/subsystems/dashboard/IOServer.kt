package com.team4099.robot2025.subsystems.dashboard

import edu.wpi.first.net.WebServer
import edu.wpi.first.networktables.BooleanPublisher
import edu.wpi.first.networktables.BooleanSubscriber
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.networktables.PubSubOption
import edu.wpi.first.wpilibj.Filesystem
import java.nio.file.Paths

class ReefControlsIOServer : ReefControlsIO {
  private val coralStateIn: BooleanSubscriber
  private val algaeStateIn: BooleanSubscriber

  private val coralStateOut: BooleanPublisher
  private val algaeStateOut: BooleanPublisher

  init {
    // Create subscribers
    val inputTable = NetworkTableInstance.getDefault().getTable(toRobotTable)
    coralStateIn =
      inputTable.getBooleanTopic(coralTopicName).subscribe(false, PubSubOption.keepDuplicates(true))
    algaeStateIn =
      inputTable.getBooleanTopic(algaeTopicName).subscribe(true, PubSubOption.keepDuplicates(true))

    // Create publishers
    val outputTable = NetworkTableInstance.getDefault().getTable(toDashboardTable)
    coralStateOut = outputTable.getBooleanTopic(coralTopicName).publish()
    algaeStateOut = outputTable.getBooleanTopic(algaeTopicName).publish()

    // Start web server
    WebServer.start(
      5801,
      Paths.get(Filesystem.getDeployDirectory().getAbsolutePath().toString(), "dashboard")
        .toString()
    )
  }

  override fun updateInputs(inputs: ReefControlsIO.ReefControlsIOInputs) {
    inputs.coralState =
      if (coralStateIn.readQueue().size > 0) booleanArrayOf(coralStateIn.get()) else booleanArrayOf(false)
    inputs.algaeState =
      if (algaeStateIn.readQueue().size > 0) booleanArrayOf(algaeStateIn.get()) else booleanArrayOf(true)
  }

  public override fun setCoralState(value: Boolean) {
    coralStateOut.set(value)
  }

  public override fun setAlgaeState(value: Boolean) {
    algaeStateOut.set(value)
  }


  companion object {
    private const val toRobotTable = "/ReefControls/ToRobot"
    private const val toDashboardTable = "/ReefControls/ToDashboard"
    private const val coralTopicName = "Coral"
    private const val algaeTopicName = "Algae"
  }
}