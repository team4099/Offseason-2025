package com.team4099.robot2025.subsystems.led

import com.team4099.robot2025.config.constants.Constants
import com.team4099.robot2025.config.constants.LedConstants.CandleState
import com.team4099.robot2025.subsystems.superstructure.Superstructure
import com.team4099.robot2025.util.CustomLogger
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.SubsystemBase
import java.util.function.Supplier

class Led(
  var gamePieceArmSupplier: Supplier<Constants.Universal.GamePiece?>,
  var isAlignedSupplier: Supplier<Boolean>,
  var isAligningSupplier: Supplier<Boolean>,
  var stateSupplier: Supplier<Superstructure.Companion.SuperstructureStates>,
  vararg candles: LedIO,
) : SubsystemBase() {
  var io = candles.toList()
  var inputs = List(io.size) { LedIO.LEDIOInputs() }

  var state = CandleState.NOTHING
  private var lastState: CandleState? = null

  override fun periodic() {
    // todo for testing
    state =
      if (DriverStation.isDisabled()) {
        if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue)
          CandleState.BLUE_DISABLED
        else CandleState.RED_DISABLED
      } else if (isAlignedSupplier.get()) CandleState.IS_ALIGNED
      else if (isAligningSupplier.get()) CandleState.IS_ALIGNING
      else if (gamePieceArmSupplier.get() == Constants.Universal.GamePiece.CORAL)
        CandleState.HAS_CORAL
      else if (gamePieceArmSupplier.get() == Constants.Universal.GamePiece.ALGAE)
        CandleState.HAS_ALGAE
      else if (stateSupplier.get() ==
        Superstructure.Companion.SuperstructureStates.GROUND_INTAKE_CORAL
      )
        CandleState.INTAKING_CORAL
      else CandleState.NOTHING

    CustomLogger.recordOutput("Led/state", state.name)

    for (instance in io.indices) {
      if (lastState != state) io[instance].turnOff()
      io[instance].setState(state)
      lastState = state

      CustomLogger.processInputs("Led/$instance", inputs[instance])
    }
  }
}
