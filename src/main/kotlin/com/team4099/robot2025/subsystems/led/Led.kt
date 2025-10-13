package com.team4099.robot2025.subsystems.led

import com.team4099.robot2025.config.constants.Constants
import com.team4099.robot2025.config.constants.LedConstants.CandleState
import com.team4099.robot2025.util.CustomLogger
import edu.wpi.first.wpilibj2.command.SubsystemBase
import java.util.function.Supplier

class Led(
  val io: LedIO,
  var gamePieceArmSupplier: Supplier<Constants.Universal.GamePiece?>,
  private val testSupplier: Supplier<Boolean>
) : SubsystemBase() {
  var inputs = LedIO.LEDIOInputs()

  var state = CandleState.NOTHING
  private var lastState: CandleState? = null

  override fun periodic() {
    CustomLogger.processInputs("Led", inputs)
    // todo for testing
    state =
      if (testSupplier.get()) CandleState.TEST
      else if (gamePieceArmSupplier.get() == Constants.Universal.GamePiece.CORAL)
       CandleState.HAS_CORAL
      else CandleState.NOTHING

    CustomLogger.recordOutput("Led/state", state.name)

    if (lastState != state) io.turnOff()
    io.setState(state)
    lastState = state
  }
}
