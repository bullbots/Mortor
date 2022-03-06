package frc.robot.commands.Intake_Commands

import edu.wpi.first.wpilibj2.command.CommandBase

import frc.robot.subsystems.Intake
/**
 * Creates a new IntakeCargo
 */
class IntakeCargos(private var intake: Intake, var intakeVel: Double) : CommandBase() {
    // Use addRequirements() here to declare subsystem dependencies.
    init { addRequirements(intake) }

    // Called when the command is initially scheduled.
    override fun initialize() { intake.set(intakeVel) }

    // Called every time the scheduler runs while the command is scheduled.
    override fun execute() {}

    override fun end(interrupted: Boolean) { intake.stop() }

    override fun isFinished(): Boolean { return false }
}