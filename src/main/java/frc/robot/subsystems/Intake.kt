package frc.robot.subsystems

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants
import frc.robot.util.SafeSparkMax

class Intake : SubsystemBase() {
    var intakeSpinner: SafeSparkMax? = null

    init {
        configureShuffleBoard()

        intakeSpinner = SafeSparkMax(Constants.INTAKE_SPINNER_PORT, CANSparkMaxLowLevel.MotorType.kBrushless)
        intakeSpinner!!.idleMode = CANSparkMax.IdleMode.kBrake

    }

    private fun configureShuffleBoard() {}

    override fun periodic() {}

    fun stop() { intakeSpinner?.set(0.0) }

}