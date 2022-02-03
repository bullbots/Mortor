package frc.robot.subsystems

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants
import frc.robot.util.SafeSparkMax

class Intake : SubsystemBase() {
    private var intake_spinner: SafeSparkMax? = null

    init {
        configureShuffleBoard()

        intake_spinner = SafeSparkMax(Constants.INTAKE_SPINNER, CANSparkMaxLowLevel.MotorType.kBrushless)

        intake_spinner!!.idleMode = CANSparkMax.IdleMode.kBrake;
    }

    /**
     * This sets the speed of the intake motors
     * @param spinner This is the value set to the motor
     */
    fun set(spinner: Double) {
        intake_spinner?.set(spinner)
    }

    // This sets the value of the Intake
    fun setIntake() { set(0.3) }

    private fun configureShuffleBoard() {}

    override fun periodic() {}

    fun stop() { set(0.0) }

}