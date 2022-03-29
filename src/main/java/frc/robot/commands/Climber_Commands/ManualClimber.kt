package frc.robot.commands.Climber_Commands

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.Climber

class ManualClimber(private val climber: Climber, private val climberVel: Double) : CommandBase() {

    // Use addRequirements() here to declare subsystem dependencies.
    init { addRequirements(climber) }

    // Called when the command is initially scheduled.
    override fun initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    override fun execute() { climber.setManual(climberVel) }

    override fun end(interrupted: Boolean) { climber.stop() }

    override fun isFinished(): Boolean { return false }

}