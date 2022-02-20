package frc.robot.subsystems

import com.ctre.phoenix.motorcontrol.NeutralMode
import edu.wpi.first.wpilibj.Counter
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants
import frc.robot.util.SafeTalonFX

class Climber : SubsystemBase() {
    var climberMotor: SafeTalonFX
    var hallEffectTop: Counter
    var hallEffectBot: Counter

    var delta = 0.0

    var currentState = State.RELEASE

    companion object{
        enum class State {
            BOTTOM, RELEASE, TOP
        }
    }

    init {
        configureShuffleBoard()

        hallEffectTop = Counter(Counter.Mode.kPulseLength)
        hallEffectBot = Counter(Counter.Mode.kPulseLength)

        hallEffectTop.setUpSource(0)
        hallEffectBot.setUpSource(1)

        hallEffectTop.setUpSourceEdge(true, true)
        hallEffectBot.setUpSourceEdge(true, true)

        climberMotor = SafeTalonFX(Constants.CLIMBER_PORT)
        climberMotor.setNeutralMode(NeutralMode.Brake)

        hallEffectTop.reset()
        hallEffectBot.reset()

    }

    private fun configureShuffleBoard() {}

    override fun periodic() {

        if(currentState == State.RELEASE) {
            if (hallEffectTop.get() > 0) {
                currentState = State.TOP
                delta = climberMotor.selectedSensorPosition
            } else if (hallEffectBot.get() > 0) {
                currentState = State.BOTTOM
                delta = climberMotor.selectedSensorPosition
            }
        } else if (currentState == State.BOTTOM) {
            if(climberMotor.selectedSensorPosition - delta > Constants.CLIMBER_LIMIT_THRESHOLD) {
                currentState = State.RELEASE
                hallEffectBot.reset()
            }
        } else if (currentState == State.TOP) {
            if (delta - climberMotor.selectedSensorPosition > Constants.CLIMBER_LIMIT_THRESHOLD) {
                currentState = State.RELEASE
                hallEffectTop.reset()
            }
        }
    }



    fun stop() { climberMotor.set(0.0) }


}