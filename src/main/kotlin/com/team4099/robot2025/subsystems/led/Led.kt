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

  override fun periodic() {
    // todo for testing
    state =
      if (testSupplier.get()) CandleState.HAS_CORAL
//      else if (gamePieceArmSupplier.get() == Constants.Universal.GamePiece.CORAL) CandleState.HAS_CORAL
      else CandleState.NOTHING

    CustomLogger.recordOutput("Led/state", state.name)

    io.setState(state)
  }
}
