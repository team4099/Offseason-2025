package com.team4099.robot2025.subsystems.dashboard

import edu.wpi.first.net.WebServer
import edu.wpi.first.networktables.BooleanArrayPublisher
import edu.wpi.first.networktables.BooleanArraySubscriber
import edu.wpi.first.networktables.BooleanPublisher
import edu.wpi.first.networktables.BooleanSubscriber
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.networktables.PubSubOption
import edu.wpi.first.networktables.StringArrayPublisher
import edu.wpi.first.networktables.StringArraySubscriber
import edu.wpi.first.wpilibj.Filesystem
import java.nio.file.Paths

class ReefControlsIOServer : ReefControlsIO {
  private val coralStateIn: BooleanArraySubscriber
  private val algaeStateIn: BooleanArraySubscriber
  private val prioritiesIn: StringArraySubscriber

  private val coralStateOut: BooleanArrayPublisher
  private val algaeStateOut: BooleanArrayPublisher
  private val prioritiesOut: StringArrayPublisher

  init {
    // Create subscribers
    val inputTable = NetworkTableInstance.getDefault().getTable(toRobotTable)
    coralStateIn =
      inputTable.getBooleanArrayTopic(coralTopicName).subscribe(BooleanArray(36), PubSubOption.keepDuplicates(true))
    algaeStateIn =
      inputTable.getBooleanArrayTopic(algaeTopicName).subscribe(BooleanArray(6), PubSubOption.keepDuplicates(true))
    prioritiesIn =
      inputTable.getStringArrayTopic(prioritiesTopicName).subscribe(arrayOf(), PubSubOption.keepDuplicates(true))

    // Create publishers
    val outputTable = NetworkTableInstance.getDefault().getTable(toDashboardTable)
    coralStateOut = outputTable.getBooleanArrayTopic(coralTopicName).publish()
    algaeStateOut = outputTable.getBooleanArrayTopic(algaeTopicName).publish()
    prioritiesOut = outputTable.getStringArrayTopic(prioritiesTopicName).publish()

    // Start web server
    WebServer.start(
      5801,//dont use 5800 because elastic is using that rn
      Paths.get(Filesystem.getDeployDirectory().getAbsolutePath().toString(), "dashboard")
        .toString()
    )
  }

  override fun updateInputs(inputs: ReefControlsIO.ReefControlsIOInputs) {
    inputs.coralState =
      if (coralStateIn.readQueue().size > 0) coralStateIn.get() else booleanArrayOf(false)
    inputs.algaeState =
      if (algaeStateIn.readQueue().size > 0) algaeStateIn.get() else booleanArrayOf(true)
    inputs.priorities =
      (if(prioritiesIn.readQueue().size > 0) arrayOf(prioritiesIn.get()) else arrayOf()) as Array<String> //idk if this is rlly safe tbh
  }

  public override fun setCoralState(value: BooleanArray) {
    coralStateOut.set(value)
  }

  public override fun setAlgaeState(value: BooleanArray) {
    algaeStateOut.set(value)
  }

  public override fun setPriorities(value: Array<String>) {
    prioritiesOut.set(value)
  }


  companion object {
    private const val toRobotTable = "/ReefControls/ToRobot"
    private const val toDashboardTable = "/ReefControls/ToDashboard"
    private const val coralTopicName = "Coral"
    private const val algaeTopicName = "Algae"
    private const val prioritiesTopicName = "Priorities"
  }
}