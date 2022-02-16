package frc.robot.subsystems

import com.ctre.phoenix.motorcontrol.NeutralMode
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants
import frc.robot.util.SafeTalonFX

class Climber : SubsystemBase() {
    var climberMotor: SafeTalonFX

    init {
        configureShuffleBoard()

        climberMotor = SafeTalonFX(Constants.CLIMBER_PORT)
        climberMotor.setNeutralMode(NeutralMode.Brake)

    }

    private fun configureShuffleBoard() {}

    override fun periodic() {}

    fun stop() { climberMotor.set(0.0) }


}