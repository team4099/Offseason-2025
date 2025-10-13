package com.team4099.robot2025.subsystems.led

import com.team4099.robot2025.config.constants.Constants
import com.team4099.robot2025.config.constants.LedConstants.CandleState
import edu.wpi.first.wpilibj2.command.SubsystemBase
import java.util.function.Supplier

class Led(
  val io: LedIO,
  private val gamePieceArmSupplier: Supplier<Constants.Universal.GamePiece>
) : SubsystemBase() {
  var inputs = LedIO.LEDIOInputs()

  var state = CandleState.NOTHING

  override fun periodic() {
    // todo for testing
    state =
      if (gamePieceArmSupplier.get() == Constants.Universal.GamePiece.CORAL) CandleState.HAS_CORAL
      else CandleState.NOTHING

    io.setState(state)
  }
}
