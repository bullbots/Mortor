package frc.robot.commands.Intake_Commands

import com.revrobotics.CANSparkMax
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.Constants
import frc.robot.subsystems.Intake
import kotlin.math.abs

class AutoArmCommand(private var intake: Intake, private var isDown: Boolean) : CommandBase() {
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
        intake.raiseLowerSpinner.pidController.setReference(targetTraj, CANSparkMax.ControlType.kSmartMotion)
    }

    override fun isFinished(): Boolean {
        val trajPoseError = targetTraj - intake.raiseLowerSpinner.encoder.position
        return if (isDown) {
            abs(trajPoseError) < 1000
        } else {
            false
        }
    }

    override fun end(interrupted: Boolean) {
        println("INFO: Intake Arm Down")
        intake.stopArm()
    }
}