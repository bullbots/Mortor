package frc.robot.subsystems

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants
import frc.robot.util.SafeSparkMax

/**
 * The intakeSpinner intakes a default value or set value
 * to spin the motor
 */
class Intake : SubsystemBase() {
    var intakeSpinner: SafeSparkMax
    var armSpinner: SafeSparkMax // TODO: Set the CAN ID to 9!!!

    init {
        configureShuffleBoard()

        intakeSpinner = SafeSparkMax(Constants.INTAKE_SPINNER_PORT, CANSparkMaxLowLevel.MotorType.kBrushless)
        armSpinner = SafeSparkMax(Constants.INTAKE_ARM_SPINNER_PORT, CANSparkMaxLowLevel.MotorType.kBrushless)

        intakeSpinner.idleMode = CANSparkMax.IdleMode.kBrake
        armSpinner.idleMode = CANSparkMax.IdleMode.kBrake
    }

    fun set(speed: Double) {
        intakeSpinner.set(speed)
        armSpinner.set(speed)
    }

    private fun configureShuffleBoard() {}

    override fun periodic() {}

    fun stop() {
        intakeSpinner.set(0.0)
        armSpinner.set(0.0)
    }

}