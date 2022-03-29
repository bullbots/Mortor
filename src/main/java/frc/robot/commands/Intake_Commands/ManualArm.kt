package frc.robot.commands.Intake_Commands

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.Intake

class ManualArm(private val intake: Intake, private val armVel: Double) : CommandBase() {

    init {
        // each subsystem used by the command must be passed into the addRequirements() method
        addRequirements(intake)
    }

    override fun initialize() { intake.raiseLowerSpinner.set(armVel) }

    override fun execute() {}

    override fun isFinished(): Boolean { return false }

    override fun end(interrupted: Boolean) { intake.raiseLowerSpinner.set(0.0) }
}
