package frc.robot.commands.Intake_Commands

import com.revrobotics.CANSparkMax
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.Constants
import frc.robot.subsystems.Intake
import kotlin.math.abs

class AutoArm(private val intake: Intake, private val isDown: Boolean) : CommandBase() {
    private var targetTraj = 0.0

    init {
        addRequirements(intake)

        targetTraj = if (isDown) {
                Constants.INTAKE_ARM_DOWN
            } else {
               Constants.INTAKE_ARM_HOLD
            }

    }

    override fun initialize() {
        println("INFO: TargetTraj: $targetTraj")
        intake.raiseLowerSpinner.pidController.setReference(targetTraj, CANSparkMax.ControlType.kSmartMotion)
    }

    override fun execute() {
//        intake.raiseLowerSpinner.pidController.setReference(targetTraj, CANSparkMax.ControlType.kSmartMotion)
    }

    override fun isFinished(): Boolean {
        val trajPoseError = targetTraj - intake.raiseLowerSpinner.encoder.position
        return if (isDown) {
            abs(trajPoseError) < 1
        } else {
            false
        }
    }

    override fun end(interrupted: Boolean) {
        println("INFO: Intake Arm Down")
        intake.stopArm()
    }
}