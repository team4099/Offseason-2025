package com.team4099.robot2025.config

import com.team4099.robot2025.config.constants.Constants
import com.team4099.robot2025.config.constants.DrivetrainConstants
import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj2.command.button.Trigger
import org.team4099.lib.joystick.XboxOneGamepad
import java.util.function.Consumer
import kotlin.math.absoluteValue

/**
 * Maps buttons on the driver and operator controllers to specific actions with meaningful variable
 * names.
 */
object ControlBoard {

  private val driver = XboxOneGamepad(Constants.Joysticks.DRIVER_PORT)
  private val operator = XboxOneGamepad(Constants.Joysticks.SHOTGUN_PORT)
  private val technician = XboxOneGamepad(Constants.Joysticks.TECHNICIAN_PORT)

  val rumbleConsumer =
    Consumer<Boolean> {
      driver.setRumble(GenericHID.RumbleType.kBothRumble, if (it) 1.0 else 0.0)
    }

  val strafe: Double
    get() = -driver.leftXAxis

  val forward: Double
    get() = -driver.leftYAxis

  val turn: Double
    get() {
      return if (driver.rightXAxis.absoluteValue < 0.90) {
        driver.rightXAxis * DrivetrainConstants.TELEOP_TURNING_SPEED_PERCENT
      } else {
        driver.rightXAxis
      }
    }

  val slowMode: Boolean
    get() = driver.rightJoystickButton && driver.leftShoulderButton

  val intakeCoral = Trigger { driver.rightTriggerAxis > 0.5 }
  val score = Trigger { driver.leftTriggerAxis > 0.5 }
  val climb = Trigger { driver.dPadUp }

  val prepL1OrProcessor = Trigger { driver.xButton }
  val prepL2OrAlgaeGround = Trigger { driver.aButton }
  val prepL3OrAlgaeReef = Trigger { driver.bButton }
  val prepL4OrBarge = Trigger { driver.yButton }

  val alignLeft = Trigger { driver.leftShoulderButton && !driver.rightShoulderButton }
  val alignRight = Trigger { driver.rightShoulderButton && !driver.leftShoulderButton }
  val alignCenter = Trigger { driver.leftShoulderButton || driver.rightShoulderButton }

  val resetGyro = Trigger { driver.startButton && driver.selectButton }
  val forceIdle = Trigger { driver.dPadDown }
  val eject = Trigger { driver.dPadLeft }
}
