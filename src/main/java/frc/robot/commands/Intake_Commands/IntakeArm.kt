package frc.robot.commands.Intake_Commands

import com.revrobotics.CANSparkMax
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.Constants
import frc.robot.subsystems.Intake

class IntakeArm(private var intake: Intake) : CommandBase() {



    init { addRequirements(intake) }

    override fun initialize() {
        intake.raiseLowerSpinner.pidController.setReference(Constants.INTAKE_ARM_UP, CANSparkMax.ControlType.kSmartMotion)
    }

    override fun execute() { }

    override fun end(interrupted: Boolean) {
        intake.stop()
    }

    override fun isFinished(): Boolean {
        return false
    }

}