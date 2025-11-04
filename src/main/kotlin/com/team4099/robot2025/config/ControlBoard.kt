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

  val driverRumbleConsumer =
    Consumer<Boolean> {
      driver.setRumble(GenericHID.RumbleType.kBothRumble, if (it) 1.0 else 0.0)
    }

  val operatorRumbleConsumer =
    Consumer<Boolean> {
      operator.setRumble(GenericHID.RumbleType.kBothRumble, if (it) 1.0 else 0.0)
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
    get() = driver.rightJoystickButton

  val intakeCoral = Trigger { driver.leftTriggerAxis > 0.5 }
  val score = Trigger { driver.rightTriggerAxis > 0.5 }
  val climbExtend = Trigger { operator.leftShoulderButton }
  val climbRetract = Trigger { operator.rightShoulderButton }

  val prepL1OrAlgaeGround = Trigger { operator.xButton }
  val prepL2OrProcessor = Trigger { operator.aButton }
  val prepL3OrAlgaeReef = Trigger { operator.bButton }
  val prepL4OrBarge = Trigger { operator.yButton }

  val alignLeft = Trigger { driver.leftShoulderButton }
  val alignRight = Trigger { driver.rightShoulderButton }

  val resetGyro = Trigger { driver.startButton && driver.selectButton }
  val forceIdle = Trigger { driver.dPadDown || operator.dPadDown || driver.xButton }
  val resetGamePieceNone = Trigger {
    operator.leftTriggerAxis > .5 && operator.rightTriggerAxis > .5
  }
  val resetGamePieceCoral = Trigger { operator.leftTriggerAxis > .5 && operator.leftJoystickButton }
  val resetGamePieceAlgae = Trigger {
    operator.leftTriggerAxis > .5 && operator.rightJoystickButton
  }
  val eject = Trigger { driver.dPadLeft || operator.dPadLeft }

  val test = Trigger { driver.dPadRight || operator.dPadRight }
  val targetObject = Trigger { driver.dPadUp }
}
