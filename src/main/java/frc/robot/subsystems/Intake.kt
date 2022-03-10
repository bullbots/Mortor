package frc.robot.subsystems

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants
import frc.robot.util.SafeSparkMax

/**
 * The intakeSpinner intakes a default value or set value
 * to spin the motor
 */
class Intake : SubsystemBase() {
    var intakeSpinner: SafeSparkMax
    var armSpinner: SafeSparkMax // CAN ID 11
    var raiseLowerSpinner: SafeSparkMax // CAN ID 9
    var loopIdx = 0

    init {
        configureShuffleBoard()

        intakeSpinner = SafeSparkMax(Constants.INTAKE_SPINNER_PORT, CANSparkMaxLowLevel.MotorType.kBrushless)
        armSpinner = SafeSparkMax(Constants.INTAKE_ARM_SPINNER_PORT, CANSparkMaxLowLevel.MotorType.kBrushless)
        raiseLowerSpinner = SafeSparkMax(Constants.RAISE_LOWER_ARM_PORT, CANSparkMaxLowLevel.MotorType.kBrushless)

        intakeSpinner.idleMode = CANSparkMax.IdleMode.kBrake
        armSpinner.idleMode = CANSparkMax.IdleMode.kBrake
        raiseLowerSpinner.idleMode = CANSparkMax.IdleMode.kBrake
    }

    fun set(speed: Double) {
        intakeSpinner.set(speed)
        armSpinner.set(speed)
    }

    fun holdArm() {
        raiseLowerSpinner.set(Constants.INTAKE_HOLD_ARM)
    }

    fun stopArm() {
        raiseLowerSpinner.set(0.0)
    }

    private fun configureShuffleBoard() {}

    override fun periodic() {
        loopIdx++
        if (loopIdx == 10) {
            loopIdx = 0
            SmartDashboard.putNumber("Arm Encoder", raiseLowerSpinner.encoder.position)
            SmartDashboard.putNumber("Arm Current", raiseLowerSpinner.outputCurrent)
//            println("INFO: Arm Encoder Pos: ${raiseLowerSpinner.encoder.position}")

        }
    }

    fun stop() {
        intakeSpinner.set(0.0)
        armSpinner.set(0.0)
        raiseLowerSpinner.set(0.0)
    }

}