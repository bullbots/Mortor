package frc.robot.commands.Intake_Commands

import com.revrobotics.CANSparkBase
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.Constants
import frc.robot.subsystems.Intake

class IntakeArmUp(private val intake: Intake) : Command() {

    init { addRequirements(intake) }

    override fun initialize() {
        intake.raiseLowerSpinner.pidController.setReference(Constants.INTAKE_ARM_UP, CANSparkBase.ControlType.kSmartMotion)
    }

    override fun execute() { }

    override fun end(interrupted: Boolean) {}

    override fun isFinished(): Boolean { return false }

}
