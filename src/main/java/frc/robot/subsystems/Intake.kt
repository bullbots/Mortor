package frc.robot.subsystems

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import edu.wpi.first.hal.DIOJNI
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.DigitalOutput
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

    companion object {
        var intakeEncoderVal = 0.0
        var isDown = false
    }

    init {
        configureShuffleBoard()

        intakeSpinner = SafeSparkMax(Constants.INTAKE_SPINNER_PORT)
        armSpinner = SafeSparkMax(Constants.INTAKE_ARM_SPINNER_PORT)
        raiseLowerSpinner = SafeSparkMax(Constants.RAISE_LOWER_ARM_PORT)

        raiseLowerSpinner.pidController.p = Constants.INTAKE_P
        raiseLowerSpinner.pidController.i = Constants.INTAKE_I
        raiseLowerSpinner.pidController.d = Constants.INTAKE_D
        raiseLowerSpinner.pidController.ff = Constants.INTAKE_FF
        raiseLowerSpinner.pidController.iZone = Constants.INTAKE_IZONE
        raiseLowerSpinner.pidController.setOutputRange(-1.0, 1.0)

        raiseLowerSpinner.pidController.setSmartMotionMaxVelocity(Constants.I_MAXRPM, Constants.I_SlotIdx)
        raiseLowerSpinner.pidController.setSmartMotionMinOutputVelocity(0.0, Constants.I_SlotIdx)
        raiseLowerSpinner.pidController.setSmartMotionMaxAccel(Constants.I_MAXRPM, Constants.I_SlotIdx)
        raiseLowerSpinner.pidController.setSmartMotionAllowedClosedLoopError(Constants.I_ALLOWED_ERROR, Constants.I_SlotIdx)

        intakeSpinner.idleMode = CANSparkMax.IdleMode.kBrake
        armSpinner.idleMode = CANSparkMax.IdleMode.kBrake
        raiseLowerSpinner.idleMode = CANSparkMax.IdleMode.kBrake

    }

    fun set(speed: Double) {
        intakeSpinner.set(speed)
        armSpinner.set(speed)
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
            SmartDashboard.putNumber("Arm Speed", raiseLowerSpinner.appliedOutput)
//            println("INFO: Arm Encoder Pos: ${raiseLowerSpinner.encoder.position}")
        }
    }

    fun resetEncoder() { intakeEncoderVal = raiseLowerSpinner.encoder.position }

    fun stop() {
        intakeSpinner.set(0.0)
        armSpinner.set(0.0)
        raiseLowerSpinner.set(0.0)
    }


}