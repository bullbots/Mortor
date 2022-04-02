package frc.robot.commands.Shooter_Commands

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.Intake

class FeedCargo(private val intake: Intake, private val intakeVel: Double) : CommandBase() {

    // each subsystem used by the command must be passed into the addRequirements() method
    init { addRequirements(intake) }

    override fun initialize() { intake.intakeSpinner.set(intakeVel) }

    override fun execute() {}

    override fun isFinished(): Boolean { return false }

    override fun end(interrupted: Boolean) { intake.intakeSpinner.set(0.0) }
}
